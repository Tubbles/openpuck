/*
 * rumble_echo_test: how many rumble commands per second an evdev client gets
 * through ff-memless and the xpad driver to an OpenPuck running the rumble
 * echo build (branch rumble-echo-test), how many are merged or dropped, and
 * how long each takes. The dongle answers every rumble packet it receives
 * with one input report:
 *
 *   ABS_Z   strong byte of the packet (xpad sends strong_magnitude / 256)
 *   ABS_RZ  weak byte of the packet (weak_magnitude / 256)
 *   keys    rumble packets received since the dongle booted, modulo 1024,
 *           bit 0 first: BTN_A BTN_B BTN_X BTN_Y BTN_TL BTN_TR BTN_SELECT
 *           BTN_START BTN_THUMBL BTN_THUMBR (pressed = 1)
 *
 * A command uploads its magnitudes as byte << 8 into the one effect this
 * program owns (EVIOCSFF) and then plays it (an EV_FF write). In mainline
 * 6.12, ml_ff_playback() calls ml_play_effects() synchronously, so
 * xpad_play_effect() queues the packet inside the write(). An upload alone,
 * while the effect plays, only rearms the ff-memless timer (ml_ff_upload()),
 * which sends one jiffy later and keeps only the last upload of that jiffy.
 * The replay length is the maximum, 65535 ms: with length 0,
 * ml_schedule_timer() rearms the timer for the current jiffy after each play
 * and the same packet goes out a second time.
 *
 * Usage: rumble_echo_test [/dev/input/eventN]
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>

#define PAD_VENDOR 0x045e
#define PAD_PRODUCT 0x028e
#define COUNTER_BITS 10
#define COUNTER_MASK ((1u << COUNTER_BITS) - 1)
#define NSEC_PER_MSEC 1000000LL
#define NSEC_PER_SEC 1000000000LL
#define ECHO_TIMEOUT_NS (500 * NSEC_PER_MSEC)
#define QUIET_NS (300 * NSEC_PER_MSEC)
#define SETTLE_LIMIT_NS (3 * NSEC_PER_SEC)
#define ECHO_QUEUE_SIZE 4096
#define BITS_PER_WORD (8 * sizeof(unsigned long))
#define WORDS_FOR(bits) (((bits) + BITS_PER_WORD - 1) / BITS_PER_WORD)
#define NO_VALUE (-1)

static const unsigned short counter_keys[COUNTER_BITS] = {
	BTN_A,	BTN_B,	    BTN_X,     BTN_Y,	   BTN_TL,
	BTN_TR, BTN_SELECT, BTN_START, BTN_THUMBL, BTN_THUMBR,
};

struct pad_state {
	int strong;
	int weak;
	unsigned int counter;
};

/* One input report in which the packet counter moved. */
struct echo {
	int strong;
	int weak;
	unsigned int advance;
	int64_t time_ns;
};

struct device {
	int fd;
	int effect_id;
	int kernel_timestamps;
	int grabbed;
	int resyncing;
	struct pad_state pending;
	struct pad_state committed;
	unsigned long overruns;
	unsigned long overrun_advance;
	unsigned long trigger_only_reports;
	unsigned long queue_overflows;
	struct echo queue[ECHO_QUEUE_SIZE];
	size_t queue_head;
	size_t queue_tail;
	unsigned char partial[sizeof(struct input_event)];
	size_t partial_size;
};

/* Outcome of one command sent in acknowledged mode. */
struct acknowledgement {
	int answered;
	int matched;
	int strong;
	int weak;
	unsigned int advance;
	unsigned int repeats;
	int64_t sent_ns;
	int64_t latency_ns;
};

struct rate_run {
	int count;
	int highest;
	int64_t *sent_ns;
	int64_t *latencies;
	char *echoed;
	size_t latency_count;
	unsigned long matched;
	unsigned long duplicates;
	unsigned long unknown;
	unsigned long out_of_order;
	unsigned long advance;
};

static int command_of_value[1 << 16];
static int next_sequence_index;

static int64_t now_ns(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * NSEC_PER_SEC + now.tv_nsec;
}

static int64_t event_time_ns(const struct input_event *event)
{
	return (int64_t)event->input_event_sec * NSEC_PER_SEC +
	       (int64_t)event->input_event_usec * 1000;
}

static double to_ms(int64_t ns)
{
	return (double)ns / NSEC_PER_MSEC;
}

static int bit_is_set(const unsigned long *bits, unsigned int bit)
{
	return (bits[bit / BITS_PER_WORD] >> (bit % BITS_PER_WORD)) & 1;
}

static int value_of(int strong, int weak)
{
	return (strong & 0xff) << 8 | (weak & 0xff);
}

/* Distinct for index < 255 * 255 and never a zero byte. */
static void sequence_value(int index, int *strong, int *weak)
{
	*strong = 1 + (index / 255) % 255;
	*weak = 1 + index % 255;
}

static int is_pad(int fd)
{
	struct input_id id;

	return ioctl(fd, EVIOCGID, &id) == 0 && id.vendor == PAD_VENDOR &&
	       id.product == PAD_PRODUCT;
}

static int has_rumble(int fd)
{
	unsigned long types[WORDS_FOR(EV_MAX + 1)] = { 0 };
	unsigned long effects[WORDS_FOR(FF_MAX + 1)] = { 0 };

	if (ioctl(fd, EVIOCGBIT(0, sizeof(types)), types) < 0 ||
	    !bit_is_set(types, EV_FF))
		return 0;
	if (ioctl(fd, EVIOCGBIT(EV_FF, sizeof(effects)), effects) < 0)
		return 0;
	return bit_is_set(effects, FF_RUMBLE);
}

static const char *find_pad(void)
{
	static char found[300];
	DIR *directory = opendir("/dev/input");
	struct dirent *entry;
	int matched = 0;

	if (!directory) {
		perror("/dev/input");
		return NULL;
	}
	while (!matched && (entry = readdir(directory))) {
		char path[300];
		int fd;

		if (strncmp(entry->d_name, "event", 5))
			continue;
		snprintf(path, sizeof(path), "/dev/input/%s", entry->d_name);
		fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) {
			printf("skipping %s: %s\n", path, strerror(errno));
			continue;
		}
		if (is_pad(fd) && has_rumble(fd)) {
			memcpy(found, path, sizeof(found));
			matched = 1;
		}
		close(fd);
	}
	closedir(directory);
	return matched ? found : NULL;
}

static int counter_bit(unsigned short code)
{
	for (int bit = 0; bit < COUNTER_BITS; bit++)
		if (counter_keys[bit] == code)
			return bit;
	return -1;
}

static int read_pad_state(int fd, struct pad_state *state)
{
	unsigned long keys[WORDS_FOR(KEY_MAX + 1)] = { 0 };
	struct input_absinfo strong, weak;

	if (ioctl(fd, EVIOCGKEY(sizeof(keys)), keys) < 0 ||
	    ioctl(fd, EVIOCGABS(ABS_Z), &strong) < 0 ||
	    ioctl(fd, EVIOCGABS(ABS_RZ), &weak) < 0)
		return -1;
	state->strong = strong.value;
	state->weak = weak.value;
	state->counter = 0;
	for (int bit = 0; bit < COUNTER_BITS; bit++)
		if (bit_is_set(keys, counter_keys[bit]))
			state->counter |= 1u << bit;
	return 0;
}

static void queue_echo(struct device *device, const struct echo *echo)
{
	size_t next = (device->queue_head + 1) % ECHO_QUEUE_SIZE;

	if (next == device->queue_tail) {
		device->queue_tail = (device->queue_tail + 1) % ECHO_QUEUE_SIZE;
		device->queue_overflows++;
	}
	device->queue[device->queue_head] = *echo;
	device->queue_head = next;
}

/*
 * A SYN_REPORT closes one report. Our own EV_FF writes also land in the
 * device's event frame and can flush one without any pad change, so only a
 * moved counter makes an echo.
 */
static void end_report(struct device *device, int64_t time_ns)
{
	struct pad_state *now = &device->pending;
	struct pad_state *before = &device->committed;

	if (now->counter != before->counter) {
		struct echo echo = {
			.strong = now->strong,
			.weak = now->weak,
			.advance = (now->counter - before->counter) &
				   COUNTER_MASK,
			.time_ns = time_ns,
		};

		queue_echo(device, &echo);
	} else if (now->strong != before->strong || now->weak != before->weak) {
		device->trigger_only_reports++;
	}
	*before = *now;
}

/* After SYN_DROPPED, the events up to the next SYN_REPORT are incomplete. */
static void resync(struct device *device)
{
	struct pad_state state;

	device->resyncing = 0;
	if (read_pad_state(device->fd, &state) < 0)
		return;
	device->overrun_advance += (state.counter - device->committed.counter) &
				   COUNTER_MASK;
	device->pending = state;
	device->committed = state;
}

static void handle_event(struct device *device, const struct input_event *event,
			 int64_t read_ns)
{
	int bit;

	if (event->type == EV_SYN && event->code == SYN_DROPPED) {
		device->resyncing = 1;
		device->overruns++;
		return;
	}
	if (event->type == EV_SYN && event->code == SYN_REPORT) {
		if (device->resyncing)
			resync(device);
		else
			end_report(device, device->kernel_timestamps ?
						   event_time_ns(event) :
						   read_ns);
		return;
	}
	if (device->resyncing)
		return;
	if (event->type == EV_ABS && event->code == ABS_Z) {
		device->pending.strong = event->value;
	} else if (event->type == EV_ABS && event->code == ABS_RZ) {
		device->pending.weak = event->value;
	} else if (event->type == EV_KEY &&
		   (bit = counter_bit(event->code)) >= 0) {
		if (event->value)
			device->pending.counter |= 1u << bit;
		else
			device->pending.counter &= ~(1u << bit);
	}
}

/* Waits up to timeout_ns for input and parses it. -1: the device is gone. */
static int read_events(struct device *device, int64_t timeout_ns)
{
	struct pollfd poll_fd = { .fd = device->fd, .events = POLLIN };
	unsigned char bytes[64 * sizeof(struct input_event)];
	struct timespec timeout;
	int64_t read_ns;
	ssize_t got;

	if (timeout_ns < 0)
		timeout_ns = 0;
	timeout.tv_sec = timeout_ns / NSEC_PER_SEC;
	timeout.tv_nsec = timeout_ns % NSEC_PER_SEC;
	if (ppoll(&poll_fd, 1, &timeout, NULL) < 0)
		return errno == EINTR ? 0 : -1;
	if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL))
		return -1;
	if (!(poll_fd.revents & POLLIN))
		return 0;
	got = read(device->fd, bytes, sizeof(bytes));
	read_ns = now_ns();
	if (got < 0)
		return (errno == EINTR || errno == EAGAIN) ? 0 : -1;
	if (got == 0)
		return -1;
	/* A read that ends inside an event leaves the rest for the next. */
	for (ssize_t index = 0; index < got; index++) {
		struct input_event event;

		device->partial[device->partial_size++] = bytes[index];
		if (device->partial_size < sizeof(event))
			continue;
		memcpy(&event, device->partial, sizeof(event));
		device->partial_size = 0;
		handle_event(device, &event, read_ns);
	}
	return 1;
}

/* Returns 1 with the oldest unread echo, 0 once deadline_ns has passed. */
static int next_echo(struct device *device, int64_t deadline_ns,
		     struct echo *echo)
{
	for (;;) {
		int64_t now;

		if (device->queue_tail != device->queue_head) {
			*echo = device->queue[device->queue_tail];
			device->queue_tail =
				(device->queue_tail + 1) % ECHO_QUEUE_SIZE;
			return 1;
		}
		now = now_ns();
		if (now >= deadline_ns)
			return 0;
		if (read_events(device, deadline_ns - now) < 0) {
			fprintf(stderr, "error: lost the device\n");
			exit(1);
		}
	}
}

/* Reads until the pad has been quiet for QUIET_NS; returns echoes skipped. */
static unsigned long settle(struct device *device)
{
	int64_t start_ns = now_ns(), quiet_ns = start_ns;
	unsigned long skipped = 0;
	struct echo echo;

	while (now_ns() - start_ns < SETTLE_LIMIT_NS &&
	       next_echo(device, quiet_ns + QUIET_NS, &echo)) {
		skipped++;
		quiet_ns = now_ns();
	}
	return skipped;
}

static void report_settle(struct device *device)
{
	unsigned long late = settle(device);

	if (late)
		printf("(skipped %lu late echoes of earlier commands)\n", late);
}

/* evdev writes whole events or fails, but a short count is still retried. */
static int write_event(int fd, const struct input_event *event)
{
	const char *bytes = (const char *)event;
	size_t done = 0;

	while (done < sizeof(*event)) {
		ssize_t wrote = write(fd, bytes + done, sizeof(*event) - done);

		if (wrote < 0 && errno == EINTR)
			continue;
		if (wrote <= 0)
			return -1;
		done += (size_t)wrote;
	}
	return 0;
}

static int upload_effect(struct device *device, int strong, int weak)
{
	struct ff_effect effect;

	memset(&effect, 0, sizeof(effect));
	effect.type = FF_RUMBLE;
	effect.id = device->effect_id;
	effect.replay.length = 0xffff;
	effect.u.rumble.strong_magnitude = strong << 8;
	effect.u.rumble.weak_magnitude = weak << 8;
	if (ioctl(device->fd, EVIOCSFF, &effect) < 0)
		return -1;
	device->effect_id = effect.id;
	return 0;
}

static int play_effect(struct device *device, int value)
{
	struct input_event play = { .type = EV_FF, .value = value };

	play.code = device->effect_id;
	return write_event(device->fd, &play);
}

static int send_command(struct device *device, int strong, int weak)
{
	if (upload_effect(device, strong, weak) < 0)
		return -1;
	return play_effect(device, 1);
}

/*
 * The first echo with another value answers the command, except an echo of
 * the previous value: that is a late or repeated packet of the last command.
 */
static int send_acknowledged(struct device *device, int strong, int weak,
			     int previous_value, struct acknowledgement *ack)
{
	struct echo echo;

	memset(ack, 0, sizeof(*ack));
	ack->sent_ns = now_ns();
	if (send_command(device, strong, weak) < 0)
		return -1;
	while (next_echo(device, ack->sent_ns + ECHO_TIMEOUT_NS, &echo)) {
		ack->advance += echo.advance;
		ack->matched = echo.strong == strong && echo.weak == weak;
		if (!ack->matched &&
		    value_of(echo.strong, echo.weak) == previous_value) {
			ack->repeats++;
			continue;
		}
		ack->answered = 1;
		ack->strong = echo.strong;
		ack->weak = echo.weak;
		ack->latency_ns = echo.time_ns - ack->sent_ns;
		break;
	}
	return 0;
}

static void print_acknowledgement(const struct acknowledgement *ack)
{
	if (!ack->answered) {
		printf("no echo within %lld ms\n",
		       ECHO_TIMEOUT_NS / NSEC_PER_MSEC);
		return;
	}
	printf("echoed %02x/%02x%s, counter +%u, %.2f ms\n", ack->strong,
	       ack->weak, ack->matched ? "" : " (wrong value)", ack->advance,
	       to_ms(ack->latency_ns));
}

static int compare_int64(const void *left, const void *right)
{
	int64_t a = *(const int64_t *)left, b = *(const int64_t *)right;

	return (a > b) - (a < b);
}

/* Nearest-rank percentile of a sorted array. */
static int64_t percentile(const int64_t *sorted, size_t count, int percent)
{
	size_t rank = (count * (size_t)percent + 99) / 100;

	return sorted[rank ? rank - 1 : 0];
}

static void print_ms_stats(const char *label, int64_t *values, size_t count)
{
	if (!count) {
		printf("%s: no samples\n", label);
		return;
	}
	qsort(values, count, sizeof(*values), compare_int64);
	printf("%s: min %.2f, median %.2f, p95 %.2f, max %.2f ms (%zu samples)\n",
	       label, to_ms(values[0]), to_ms(percentile(values, count, 50)),
	       to_ms(percentile(values, count, 95)), to_ms(values[count - 1]),
	       count);
}

static int probe(struct device *device)
{
	for (int attempt = 0; attempt < 3; attempt++) {
		struct acknowledgement ack;
		int strong, weak;

		sequence_value(next_sequence_index++, &strong, &weak);
		if (send_acknowledged(device, strong, weak, NO_VALUE, &ack) <
		    0) {
			fprintf(stderr, "error: rumble command failed: %s\n",
				strerror(errno));
			return -1;
		}
		if (ack.answered) {
			printf("probe: sent %02x/%02x, ", strong, weak);
			print_acknowledgement(&ack);
			return 0;
		}
	}
	fprintf(stderr, "error: no echo to 3 rumble commands; is the dongle "
			"running the rumble echo build?\n");
	/* input_inject_event() drops writes from all but the grabbing client. */
	if (!device->grabbed)
		fprintf(stderr, "another process may hold an EVIOCGRAB on the "
				"pad, which discards our EV_FF writes\n");
	return -1;
}

/*
 * 256 commands with every strong byte, then 256 with every weak byte, each
 * paired with a permutation of the other motor, never 00/00 and never the
 * same value twice in a row.
 */
static void sweep_value(int index, int *strong, int *weak)
{
	int step = index % 256;

	if (index < 256) {
		*strong = step;
		*weak = (step * 167 + 89) & 0xff;
	} else {
		*strong = (step * 101 + 37) & 0xff;
		*weak = step;
	}
}

static void print_bytes_seen(const char *label, const char *seen)
{
	int missing = 0;

	for (int byte = 0; byte < 256; byte++)
		missing += !seen[byte];
	printf("%s bytes echoed correctly: %d of 256\n", label, 256 - missing);
	if (!missing)
		return;
	printf("  never echoed correctly:");
	for (int byte = 0; byte < 256; byte++)
		if (!seen[byte])
			printf(" %02x", byte);
	printf("\n");
}

static void test_value_sweep(struct device *device)
{
	enum { COMMANDS = 512, SHOWN = 10 };
	int64_t latencies[COMMANDS];
	char strong_seen[256] = { 0 }, weak_seen[256] = { 0 };
	size_t latency_count = 0;
	int correct = 0, wrong = 0, timeouts = 0, send_errors = 0;
	int stepped_one = 0, stepped_more = 0;
	unsigned long repeats = 0;
	int previous = NO_VALUE;

	printf("\n== Value sweep, acknowledged ==\n");
	report_settle(device);
	for (int index = 0; index < COMMANDS; index++) {
		struct acknowledgement ack;
		int strong, weak;

		sweep_value(index, &strong, &weak);
		if (send_acknowledged(device, strong, weak, previous, &ack) <
		    0) {
			send_errors++;
			continue;
		}
		previous = value_of(strong, weak);
		repeats += ack.repeats;
		if (!ack.answered) {
			timeouts++;
			continue;
		}
		if (ack.advance == 1)
			stepped_one++;
		else
			stepped_more++;
		strong_seen[strong] |= ack.strong == strong;
		weak_seen[weak] |= ack.weak == weak;
		if (!ack.matched) {
			if (wrong++ < SHOWN)
				printf("wrong echo: sent %02x/%02x, got %02x/%02x\n",
				       strong, weak, ack.strong, ack.weak);
			continue;
		}
		correct++;
		latencies[latency_count++] = ack.latency_ns;
	}
	printf("%d commands (strong/weak, no 00/00): correct %d, wrong value %d, "
	       "timeouts %d, send errors %d\n",
	       COMMANDS, correct, wrong, timeouts, send_errors);
	printf("counter advanced by exactly one: %d, by more: %d\n",
	       stepped_one, stepped_more);
	printf("extra echoes repeating the previous value: %lu\n", repeats);
	print_bytes_seen("strong", strong_seen);
	print_bytes_seen("weak", weak_seen);
	print_ms_stats("latency, send to echo", latencies, latency_count);
}

static void count_rate_echo(struct rate_run *run, const struct echo *echo)
{
	int command = command_of_value[value_of(echo->strong, echo->weak)];

	run->advance += echo->advance;
	if (command < 0) {
		run->unknown++;
	} else if (run->echoed[command]) {
		run->duplicates++;
	} else {
		run->echoed[command] = 1;
		run->matched++;
		run->latencies[run->latency_count++] =
			echo->time_ns - run->sent_ns[command];
		if (command < run->highest)
			run->out_of_order++;
		else
			run->highest = command;
	}
}

static void print_rate_run(const struct rate_run *run, int rate,
			   int64_t elapsed_ns, unsigned long send_errors,
			   unsigned long hidden)
{
	printf("%d/s: sent %d over %.2f s (%.1f/s), send errors %lu\n", rate,
	       run->count, to_ms(elapsed_ns) / 1000,
	       elapsed_ns > 0 ? (run->count - 1) * 1e9 / elapsed_ns : 0.0,
	       send_errors);
	printf("  echoed %lu, never echoed (merged or dropped) %lu, "
	       "out of order %lu\n",
	       run->matched, run->count - run->matched, run->out_of_order);
	printf("  dongle received %lu packets (counter advance), "
	       "duplicate echoes %lu, echoes of unknown values %lu\n",
	       run->advance + hidden, run->duplicates, run->unknown);
}

static void run_rate(struct device *device, int rate)
{
	struct rate_run run = { .count = 2 * rate, .highest = -1 };
	int64_t period_ns = NSEC_PER_SEC / rate, start_ns, last_ns, quiet_ns;
	unsigned long hidden = device->overrun_advance, send_errors = 0;
	struct echo echo;

	run.sent_ns = calloc(run.count, sizeof(*run.sent_ns));
	run.latencies = calloc(run.count, sizeof(*run.latencies));
	run.echoed = calloc(run.count, 1);
	if (!run.sent_ns || !run.latencies || !run.echoed) {
		printf("%d/s: out of memory\n", rate);
		goto out;
	}
	report_settle(device);
	memset(command_of_value, 0xff, sizeof(command_of_value));
	start_ns = now_ns();
	for (int command = 0; command < run.count; command++) {
		int strong, weak;

		while (next_echo(device, start_ns + command * period_ns, &echo))
			count_rate_echo(&run, &echo);
		sequence_value(next_sequence_index++, &strong, &weak);
		command_of_value[value_of(strong, weak)] = command;
		run.sent_ns[command] = now_ns();
		if (send_command(device, strong, weak) < 0)
			send_errors++;
	}
	last_ns = now_ns();
	quiet_ns = last_ns;
	while (now_ns() - last_ns < SETTLE_LIMIT_NS &&
	       next_echo(device, quiet_ns + QUIET_NS, &echo)) {
		count_rate_echo(&run, &echo);
		quiet_ns = now_ns();
	}
	print_rate_run(&run, rate, last_ns - start_ns, send_errors,
		       device->overrun_advance - hidden);
	print_ms_stats("  latency, send to echo", run.latencies,
		       run.latency_count);
out:
	free(run.sent_ns);
	free(run.latencies);
	free(run.echoed);
}

static void test_rates(struct device *device)
{
	static const int rates[] = { 50, 100, 200, 400, 800 };

	printf("\n== Rate sweep, open loop, 2 s per rate ==\n");
	for (size_t index = 0; index < sizeof(rates) / sizeof(rates[0]);
	     index++)
		run_rate(device, rates[index]);
}

static void test_max_throughput(struct device *device)
{
	unsigned long correct = 0, wrong = 0, timeouts = 0, send_errors = 0;
	unsigned long repeats = 0;
	int previous = NO_VALUE;
	int64_t start_ns, elapsed_ns;

	printf("\n== Maximum acknowledged throughput, 3 s ==\n");
	report_settle(device);
	start_ns = now_ns();
	while (now_ns() - start_ns < 3 * NSEC_PER_SEC) {
		struct acknowledgement ack;
		int strong, weak;

		sequence_value(next_sequence_index++, &strong, &weak);
		if (send_acknowledged(device, strong, weak, previous, &ack) <
		    0) {
			send_errors++;
			continue;
		}
		previous = value_of(strong, weak);
		repeats += ack.repeats;
		if (!ack.answered)
			timeouts++;
		else if (!ack.matched)
			wrong++;
		else
			correct++;
	}
	elapsed_ns = now_ns() - start_ns;
	printf("%lu commands echoed correctly in %.2f s: %.1f per second\n",
	       correct, to_ms(elapsed_ns) / 1000, correct * 1e9 / elapsed_ns);
	printf("wrong value %lu, timeouts %lu, send errors %lu, "
	       "extra echoes repeating the previous value %lu\n",
	       wrong, timeouts, send_errors, repeats);
}

static void test_messages(struct device *device)
{
	enum { MESSAGES = 200, PARTS = 5 };
	int64_t durations[MESSAGES];
	size_t duration_count = 0;
	unsigned long failed = 0;
	int previous = NO_VALUE;

	printf("\n== Multi-command message, %d messages of %d commands ==\n",
	       MESSAGES, PARTS);
	report_settle(device);
	for (int message = 0; message < MESSAGES; message++) {
		int64_t start_ns = 0, end_ns = 0;
		int complete = 1;

		for (int part = 0; part < PARTS; part++) {
			struct acknowledgement ack;
			int strong, weak;

			sequence_value(next_sequence_index++, &strong, &weak);
			if (send_acknowledged(device, strong, weak, previous,
					      &ack) < 0 ||
			    !ack.matched)
				complete = 0;
			previous = value_of(strong, weak);
			if (part == 0)
				start_ns = ack.sent_ns;
			end_ns = ack.sent_ns + ack.latency_ns;
		}
		if (complete)
			durations[duration_count++] = end_ns - start_ns;
		else
			failed++;
	}
	printf("complete %zu, failed %lu (a command timed out or echoed a "
	       "wrong value)\n",
	       duration_count, failed);
	print_ms_stats("time per message, first send to last echo", durations,
		       duration_count);
	if (duration_count)
		printf("%d payload bits per message: %.0f bit/s at the median\n",
		       PARTS * 16,
		       PARTS * 16 * 1e9 /
			       percentile(durations, duration_count, 50));
}

static void test_zero(struct device *device)
{
	static const int steps[][2] = {
		{ 0x40, 0x40 }, { 0x00, 0x00 }, { 0x00, 0x00 },
		{ 0x40, 0x00 }, { 0x00, 0x40 }, { 0x41, 0x41 },
	};
	int previous = NO_VALUE;

	printf("\n== Zero magnitude, acknowledged ==\n");
	report_settle(device);
	for (size_t index = 0; index < sizeof(steps) / sizeof(steps[0]);
	     index++) {
		struct acknowledgement ack;
		int strong = steps[index][0], weak = steps[index][1];

		printf("sent %02x/%02x: ", strong, weak);
		if (send_acknowledged(device, strong, weak, previous, &ack) < 0)
			printf("send failed: %s\n", strerror(errno));
		else
			print_acknowledgement(&ack);
		previous = value_of(strong, weak);
	}
}

/* Stopping a playing effect makes ff-memless send one all-zero packet. */
static void stop_and_remove(struct device *device)
{
	struct echo echo;
	int64_t sent_ns;

	printf("\n== Stop and remove ==\n");
	report_settle(device);
	sent_ns = now_ns();
	if (play_effect(device, 0) < 0)
		printf("stop failed: %s\n", strerror(errno));
	else if (next_echo(device, sent_ns + ECHO_TIMEOUT_NS, &echo))
		printf("stop: echoed %02x/%02x, counter +%u, %.2f ms\n",
		       echo.strong, echo.weak, echo.advance,
		       to_ms(echo.time_ns - sent_ns));
	else
		printf("stop: no echo within %lld ms\n",
		       ECHO_TIMEOUT_NS / NSEC_PER_MSEC);
	if (ioctl(device->fd, EVIOCRMFF, device->effect_id) < 0)
		printf("EVIOCRMFF failed: %s\n", strerror(errno));
	else
		printf("effect %d removed\n", device->effect_id);
}

static void print_totals(const struct device *device)
{
	printf("\n== Totals ==\n");
	printf("evdev overruns (SYN_DROPPED) %lu, packets they hid %lu\n",
	       device->overruns, device->overrun_advance);
	printf("reports that moved a trigger but not the counter: %lu\n",
	       device->trigger_only_reports);
	printf("echo queue overflows in this program: %lu\n",
	       device->queue_overflows);
}

static void print_header(struct device *device, const char *path)
{
	char name[256] = "";
	struct input_id id = { 0 };
	struct utsname kernel = { 0 };
	int effects = 0;

	ioctl(device->fd, EVIOCGNAME(sizeof(name) - 1), name);
	ioctl(device->fd, EVIOCGID, &id);
	ioctl(device->fd, EVIOCGEFFECTS, &effects);
	uname(&kernel);
	printf("device %s \"%s\", bus %04x, id %04x:%04x, version %04x\n", path,
	       name, id.bustype, id.vendor, id.product, id.version);
	printf("kernel %s %s %s, %d effect slots\n", kernel.sysname,
	       kernel.release, kernel.machine, effects);
	if (!is_pad(device->fd))
		printf("warning: not %04x:%04x, an echo is unlikely\n",
		       PAD_VENDOR, PAD_PRODUCT);
}

static int setup(struct device *device, const char *path)
{
	struct input_event gain = { .type = EV_FF,
				    .code = FF_GAIN,
				    .value = 0xffff };
	int clock_id = CLOCK_MONOTONIC;

	device->fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (device->fd < 0) {
		fprintf(stderr, "error: %s: %s\n", path, strerror(errno));
		return -1;
	}
	if (!has_rumble(device->fd)) {
		fprintf(stderr, "error: %s has no FF_RUMBLE\n", path);
		return -1;
	}
	print_header(device, path);
	device->kernel_timestamps =
		ioctl(device->fd, EVIOCSCLOCKID, &clock_id) == 0;
	printf("echo time: %s\n",
	       device->kernel_timestamps ?
		       "kernel event timestamp (CLOCK_MONOTONIC)" :
		       "when read() returned (EVIOCSCLOCKID failed)");
	/* Without the grab the TV also sees the echoed button presses. */
	device->grabbed = ioctl(device->fd, EVIOCGRAB, 1) == 0;
	if (!device->grabbed)
		printf("warning: EVIOCGRAB failed (%s), other readers see the "
		       "echoed buttons\n",
		       strerror(errno));
	/* ff-memless scales every magnitude by gain / 0xffff. */
	if (write_event(device->fd, &gain) < 0) {
		fprintf(stderr, "error: setting FF_GAIN: %s\n",
			strerror(errno));
		return -1;
	}
	if (upload_effect(device, 0, 0) < 0) {
		fprintf(stderr, "error: EVIOCSFF: %s\n", strerror(errno));
		return -1;
	}
	if (read_pad_state(device->fd, &device->committed) < 0) {
		fprintf(stderr, "error: reading the pad state: %s\n",
			strerror(errno));
		return -1;
	}
	device->pending = device->committed;
	settle(device);
	printf("start: counter %u, strong %02x, weak %02x\n",
	       device->committed.counter, device->committed.strong,
	       device->committed.weak);
	return 0;
}

int main(int argc, char **argv)
{
	static struct device device = { .fd = -1, .effect_id = -1 };
	const char *path;

	setvbuf(stdout, NULL, _IOLBF, 0);
	if (argc > 2) {
		fprintf(stderr, "usage: %s [/dev/input/eventN]\n", argv[0]);
		return 1;
	}
	path = argc == 2 ? argv[1] : find_pad();
	if (!path) {
		fprintf(stderr,
			"error: no %04x:%04x device with FF_RUMBLE in "
			"/dev/input\n",
			PAD_VENDOR, PAD_PRODUCT);
		return 1;
	}
	if (setup(&device, path) < 0 || probe(&device) < 0)
		return 1;
	test_value_sweep(&device);
	test_rates(&device);
	test_max_throughput(&device);
	test_messages(&device);
	test_zero(&device);
	stop_and_remove(&device);
	print_totals(&device);
	ioctl(device.fd, EVIOCGRAB, 0);
	close(device.fd);
	return 0;
}
