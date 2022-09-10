/*
*   CSE438 Embedded Systems Programming Spring 2022
*   Assignment 2: PWM Operation and Distance Measurement on RPi
*/

#include <stdio.h>    // for printf
#include <fcntl.h>    // for open
#include <unistd.h> 
#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <gpiod.h>
#include <stdint.h>
#include "project2.h"

#define PWMCHIP0_EXPORT_PATH        "/sys/class/pwm/pwmchip0/export"
#define PWMCHIP0_UNEXPORT_PATH      "/sys/class/pwm/pwmchip0/unexport"

#define US_TO_NS                    1000
#define MS_TO_NS                    1000000
#define S_TO_NS                     1000000000

#define PERIOD                      (PWM_PERIOD * MS_TO_NS)

#define TRIG_PERIOD                 (10 * US_TO_NS)

#define RED_LED                     0                   // PWMCHIP0 Channel 0
#define GREEN_LED                   25                  // GPIO 25
#define BLUE_LED                    1                   // PWMCHIP0 Channel 1

#define TRIG_PIN                    22                  // GPIO 22
#define ECHO_PIN                    23                  // GPIO 23

#define ENABLE                      1
#define DISABLE                     0

// Maximum distance sensor can measure in inches
#define MAX_DIST_INCH               380

// Maximum distance sensor can measure in centimeters
#define MAX_DIST_CM                 1000

timer_t pwm_timer_id;

bool bitG = 0, ledR_on, ledG_on, ledB_on;
bool running;

char command[50];

int expire_count = 0, sec_count = 0;

int loop_i = 0;

// Get the R, G and B duty from user
int red_led_duty;
int green_led_duty;
int blue_led_duty;

// Get the Measurement counts and mode from user
int measurement_counts;
int measurement_mode;

uint32_t t0, t1;
uint32_t curr_cpu_freq;

double distance, average_distance;
double elapsed;

struct gpiod_chip *chip;
struct gpiod_line *lineG;

struct gpiod_line *line_trig, *line_echo;

int pwmchip0_enable(int, int);

void timer_callback(int sig_no)
{
    if(sig_no == SIGUSR1)
    {
        ++expire_count;
        ++sec_count;
        if(expire_count == PWM_PERIOD)
        {
            expire_count = 0;
            if(green_led_duty > 0)
                bitG = 1;
        }

        if(ledG_on)
        {
            if(green_led_duty <= 0)
            {
                bitG = 0;
            }
            else if(green_led_duty >= 100)
            {
                bitG = 1;
            }
            else if(expire_count == ((green_led_duty * PWM_PERIOD) / 100))
            {
                bitG = 0;
            }
        }
        else
        {
            bitG = 0;
        }

        if(sec_count == STEP_DURATION)
        {
            sec_count = 0;

            ledR_on = loop_i & 1;
            ledG_on = (loop_i >> 1) & 1;
            ledB_on = (loop_i >> 2) & 1;

            if(ledR_on)
                pwmchip0_enable(RED_LED, ENABLE);
            else
                pwmchip0_enable(RED_LED, DISABLE);

            if(ledB_on)
                pwmchip0_enable(BLUE_LED, ENABLE);
            else
                pwmchip0_enable(BLUE_LED, DISABLE);

            loop_i++;
        }
    }
    gpiod_line_set_value(lineG, bitG);
}

int pwmchip0_export_channel(int channel)
{
    int fd;
    char channel_str[2];
    sprintf(channel_str, "%d", channel);
    printf("Exporting Channel: %s\n", channel_str);
    fd = open(PWMCHIP0_EXPORT_PATH, O_WRONLY);
    if(fd == -1)
    {
        printf("Error exporting pwmchip0 channel %d\n", channel);
        return -1;
    }
    if(write(fd, &channel_str, 1) == -1)
        printf("Error writing to export file\n");
    close(fd);
    return 0;
}

int pwmchip0_unexport_channel(int channel)
{
    int fd;
    char channel_str[2];
    sprintf(channel_str, "%d", channel);
    printf("Unexporting Channel: %s\n", channel_str);
    fd = open(PWMCHIP0_UNEXPORT_PATH, O_WRONLY);
    if(fd == -1)
    {
        printf("Error unexporting pwmchip0 channel %d\n", channel);
        return -1;
    }
    if(write(fd, &channel_str, 1) == -1)
        printf("Error writing to unexport file\n");
    close(fd);
    return 0;
}

int pwmchip0_enable(int channel, int value)
{
    int fd;
    char path[256], write_buffer[20];
    ssize_t bytes_written;

    sprintf(path, "/sys/class/pwm/pwmchip0/pwm%d/enable", channel);
    // printf("Path to enable: %s\n", path);

    fd = open(path, O_WRONLY);
    if(fd == -1)
        return -1;

    bytes_written = snprintf(write_buffer, 20, "%d", value);
    // printf("Write buffer: %s with %d bytes\n", write_buffer, bytes_written);
    if(write(fd, write_buffer, bytes_written) == -1)
        printf("Cannot enable\n");

    close(fd);
    return 0;
}

int pwmchip0_setPeriod(int channel, int period)
{
    int fd;
    char path[256], write_buffer[20];
    ssize_t bytes_written;

    sprintf(path, "/sys/class/pwm/pwmchip0/pwm%d/period", channel);
    // printf("Path for Channel %d period: %s\n", channel, path);

    fd = open(path, O_WRONLY);
    if(fd == -1)
        return -1;

    bytes_written = snprintf(write_buffer, 20, "%d", period);
    // printf("Write buffer: %s with %d bytes\n", write_buffer, bytes_written);
    if(write(fd, write_buffer, bytes_written) == -1)
        printf("Cannot set period\n");

    close(fd);
    return 0;
}

int pwmchip0_setDuty(int channel, int duty)
{
    int fd;
    char path[256], write_buffer[20];
    ssize_t bytes_written;

    sprintf(path, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", channel);
    // printf("Path for %d channel's period: %s\n", channel, path);

    fd = open(path, O_WRONLY);
    if(fd == -1)
        return -1;

    bytes_written = snprintf(write_buffer, 20, "%d", duty);
    // printf("Write buffer: %s with %d bytes\n", write_buffer, bytes_written);
    if(write(fd, write_buffer, bytes_written) == -1)
        printf("Cannot set duty\n");

    close(fd);
    return 0;
}

void start_rgb_sequence()
{
    if(!running)
    {
        // Create signal event to raise a User Signal whenever the timer expires
        struct sigevent sig_evt;
        sig_evt.sigev_notify = SIGEV_SIGNAL;
        sig_evt.sigev_signo = SIGUSR1;
        sig_evt.sigev_value.sival_ptr = &pwm_timer_id;
        signal(SIGUSR1, timer_callback);

        // Create a timer to generate software pwm
        timer_create(CLOCK_MONOTONIC, &sig_evt, &pwm_timer_id);

        // Load the timer which expires every 1 ms
        struct itimerspec itimer;
        itimer.it_value.tv_sec = 0;
        itimer.it_value.tv_nsec = MS_TO_NS;
        itimer.it_interval.tv_sec = itimer.it_value.tv_sec;
        itimer.it_interval.tv_nsec = itimer.it_value.tv_nsec;

        // Start the timer. It generates callback whenever timer period expires
        timer_settime(pwm_timer_id, 0, &itimer, NULL);
    }

    // Set the duty cycle based on used input
    if(pwmchip0_setDuty(RED_LED, ((red_led_duty * PERIOD) / 100)))
        printf("Cannot open Red duty\n");
    if(pwmchip0_setDuty(BLUE_LED, ((blue_led_duty * PERIOD) / 100)))
        printf("Cannot open Green duty\n");
}

void stop_rgb_sequence()
{
    // This stops and deletes the created timer
    timer_delete(pwm_timer_id);
}

void start_distance_sensor()
{
    // Give a 10us pulse to start transmission
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    gpiod_line_set_value(line_trig, 1);

    next.tv_sec += ((next.tv_nsec + TRIG_PERIOD) / S_TO_NS);
    next.tv_nsec = ((next.tv_nsec + TRIG_PERIOD) % S_TO_NS);

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    
    gpiod_line_set_value(line_trig, 0);
}

uint32_t get_cpu_current_frequency()
{
    // Get the current CPU Frequency for CCNT Calculations
    int fd;
    char freq_val[20];
    ssize_t bytes_read;

    fd = open("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq", O_RDONLY);
    if(fd == -1)
        return -1;
    
    bytes_read = read(fd, &freq_val, sizeof(freq_val));
    if(bytes_read == -1)
    {
        close(fd);
        return -1;
    }

    close(fd);
    return (uint32_t)atoi(freq_val);
}

// Function to Subtract ts2 from ts1. The result is in ts3
void ts_sub(struct timespec *ts1, struct timespec *ts2, struct timespec *ts3)
{
    ts3->tv_sec = ts1->tv_sec - ts2->tv_sec;
    
    if(ts1->tv_nsec >= ts2->tv_nsec) {
		ts3->tv_nsec = ts1->tv_nsec - ts2->tv_nsec;	
	} else
	{	
		ts3->tv_sec--; 
		ts3->tv_nsec = 1e9 + ts1->tv_nsec - ts2->tv_nsec;	
    }
}

// Read Cycle Counts in the User Space.
static inline uint32_t ccnt_read (void)
{
    uint32_t cc = 0;
    __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0":"=r" (cc));
    return cc;
}

int main(void)
{
    int req;
    
    // Timespec structure used to calculate time befween distance sensor's echo pulses
    struct timespec start, end, result;

    // Open gpiochip0 to access the RPi's GPIO pins
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip)
    {
        printf("Failed to open GPIO Chip\n");
        return -1;
    }

    // Get the Green LED's GPIO Line
    lineG = gpiod_chip_get_line(chip, GREEN_LED);	
    if (!lineG)
    {
        printf("Failed to get GPIO Line\n");
        gpiod_chip_close(chip);
        return -1;
    }

    // Get the Trigger Pin's GPIO Line
    line_trig = gpiod_chip_get_line(chip, TRIG_PIN);
    // Get the Echo Pin's GPIO Line
    line_echo = gpiod_chip_get_line(chip, ECHO_PIN);

    // Set the Green LED pin a output pin
    req = gpiod_line_request_output(lineG, "gpio_state", GPIOD_LINE_ACTIVE_STATE_LOW);
    printf("Get Lines & Output: %d \n", req);
    if (req)
    {
        gpiod_chip_close(chip);
        return -1;
    }

    // Set the Trigger pin a output pin
    req = gpiod_line_request_output(line_trig, "trig_state", GPIOD_LINE_ACTIVE_STATE_LOW);
    // Set the Echo pin a input pin with event detection on both edges
    req = gpiod_line_request_both_edges_events(line_echo, "echo_state");

    gpiod_line_set_value(lineG, bitG);
    gpiod_line_set_value(line_trig, 0);

    // Export Red and Blue LED Channels
    if(!pwmchip0_export_channel(RED_LED))
        printf("Channel 0 Exported\n");
    if(!pwmchip0_export_channel(BLUE_LED))
        printf("Channel 10 Exported\n");

    // Set the period for Red and Blue LED Channels
    if(pwmchip0_setPeriod(RED_LED, PERIOD) == -1)
        printf("Cannot open period\n");
    if(pwmchip0_setPeriod(BLUE_LED, PERIOD) == -1)
        printf("Cannot open period\n");

    // Set the duty cycle for Red and Blue LED Channels
    if(pwmchip0_setDuty(RED_LED, 0))
        printf("Cannot open duty\n");
    if(pwmchip0_setDuty(BLUE_LED, 0))
        printf("Cannot open duty\n");

    while(1)
    {
        printf("\nEnter your command\n");
        // Wait for user command
        scanf("%s", command);

        // Check the user input command with available commands
        if(!strcmp(command, "exit"))
            break;
        else if(!strcmp(command, "rgb"))
        {
            // For 'rgb' command, get the duty cycle for R, G and B LEDs
            scanf("%d", &red_led_duty);
            scanf("%d", &green_led_duty);
            scanf("%d", &blue_led_duty);
            // Start the timer if not already, set the duty cycle for R, G and B Led
            start_rgb_sequence();
            printf("New Red Duty: %d\n", red_led_duty);
            printf("New Green Duty: %d\n", green_led_duty);
            printf("New Blue Duty: %d\n", blue_led_duty);

            if(!running)
                running = 1;
        }
        else if(!strcmp(command, "dist"))
        {
            // For 'dist' command, get number of measurements and calculation mode
            scanf("%d", &measurement_counts);
            scanf("%d", &measurement_mode);

            average_distance = 0;

            for(int i=0; i<(measurement_counts + 2); i++)
            {
                // Start the measurement
                start_distance_sensor();

                // Wait to receive rising edge on Echo Pin
                while(!gpiod_line_get_value(line_echo));
                // Get the time during rising edge
                if(measurement_mode == 0)
                    clock_gettime(CLOCK_MONOTONIC, &start);
                else
                    t0 = ccnt_read();

                // Wait to receive falling edge on Echo Pin
                while(gpiod_line_get_value(line_echo));
                // Get the time during falling edge
                if(measurement_mode == 0)
                    clock_gettime(CLOCK_MONOTONIC, &end);
                else
                    t1 = ccnt_read();

                // This condition discards the first two measurements for better response
                if(i >= 2)
                {
                    // Calculate the time between start and end
                    if(measurement_mode == 0)
                    {
                        ts_sub(&end, &start, &result);
                        elapsed = ((result.tv_sec*1e9 + result.tv_nsec) / 2);
                    }
                    else
                    {
                        // Get the current CPU Frequency to perform accurate calculations
                        curr_cpu_freq = get_cpu_current_frequency();
                        if(curr_cpu_freq == -1)
                            printf("Error reading CPU Frequency");
                        // else
                        //     printf("CPU Frequency: %u\n", curr_cpu_freq);
                        elapsed = ((double)(t1 - t0) / ((double)curr_cpu_freq / (double)MS_TO_NS));
                        elapsed /= 2;
                    }

                    // Calculate the distance on basis of sound speed * elapsed time
                    distance = ((double)(elapsed * 13503.9) / (double)S_TO_NS);

                    // Filter out the dummy or noisy distances
                    if(distance > MAX_DIST_INCH)
                    {
                        i--;
                        continue;
                    }

                    // Print the correct distance
                    printf("Distance %d: %f inch\n", (i-1),  distance);
                    average_distance += distance;
                }
                // usleep(1000);
            }
            // Calculate the average distance
            average_distance /= (double)measurement_counts;
            printf("\nAverage Distance: %.2f inch\n", average_distance);
        }
        else
        {
            printf("Invalid Command. Please enter command 'rgb' with 3 arguments or 'dist' with 2 arguments\n");
        }
    }

    // Stop all the timers and close all the open/exported file and terminate gracefully
    if(running)
    {
        running = 0;
        stop_rgb_sequence();
    }

	gpiod_chip_close(chip);

    pwmchip0_enable(RED_LED, DISABLE);
    pwmchip0_enable(BLUE_LED, DISABLE);

    if(!pwmchip0_unexport_channel(RED_LED))
        printf("Channel 0 Unexported\n");
    if(!pwmchip0_unexport_channel(BLUE_LED))
        printf("Channel 1 Unexported\n");

    return 0;
}
