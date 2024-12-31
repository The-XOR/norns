#include "ssd1322.h"

#include <string.h>

#include "events.h"
#include "event_types.h"

static int spidev_fd = 0;
static bool display_dirty = false;
static bool should_turn_on = true;
static uint8_t * spidev_buffer = NULL;
static struct gpiod_chip * gpio_0;
static struct gpiod_line * gpio_dc;
static struct gpiod_line * gpio_reset;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t ssd1322_pthread_t;

#define ST7789_Portrait         0xC0
#define ST7789_Portrait180      0
#define ST7789_Landscape        0xA0
#define ST7789_Landscape180     0x60

#define SPIDEV_BUFFER_LEN      (ST7789_HEIGHT * ST7789_WIDTH * sizeof(uint16_t )) // 2 bytes per pixel
#define MIN(a,b)    ((a)<(b) ? (a) : (b))

int open_spi() 
{
    int spi_fd = open(SPIDEV_0_0_PATH, O_RDWR | O_SYNC);
    if( spi_fd < 0 )
    {
        fprintf(stderr, "(screen) couldn't open %s\n", SPIDEV_0_0_PATH);
        return -1;
    }
    
 	uint8_t mode = SPI_MODE_0;
    uint8_t bits_per_word = 8;
    uint32_t speed_hz = 40000000;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 
        || ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        fprintf(stderr, "could not set SPI WR settings via IOC\n");
        close(spi_fd);
        spi_fd = -1;
    }
    return spi_fd;
}

int ssd1322_write_command(uint8_t command, uint8_t data_len, ...) 
{
    va_list args;
    uint8_t cmd_buf[1];
    uint8_t data_buf[256];
    struct spi_ioc_transfer cmd_transfer = {0};
    struct spi_ioc_transfer data_transfer = {0};

    pthread_mutex_lock(&lock);

    if( spidev_fd <= 0 ){
        fprintf(stderr, "%s: spidev not yet opened\n", __func__);
        goto fail;
    }

    gpiod_line_set_value(gpio_dc, 0);

    cmd_buf[0] = command;
    cmd_transfer.tx_buf = (unsigned long) cmd_buf;
    cmd_transfer.len = (uint32_t) sizeof(cmd_buf);

    if( ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &cmd_transfer) < 0 ){
        fprintf(stderr, "%s: could not send command-message.\n", __func__);
        goto fail;
    }

    if( data_len > 0 ){
        gpiod_line_set_value(gpio_dc, 1);

        va_start(args, data_len);

        for( uint8_t i = 0; i < data_len; i++ ){
            data_buf[i] = va_arg(args, int);
        }

        va_end(args);

        data_transfer.tx_buf = (unsigned long) data_buf;
        data_transfer.len = (uint32_t) data_len;

        if( ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &data_transfer) < 0 ){
            fprintf(stderr, "%s: could not send data-message.\n", __func__);
            goto fail;
        }
    }

    pthread_mutex_unlock(&lock);
    return 0;
    fail:
    pthread_mutex_unlock(&lock);
    return -1;
}

int write_data(uint8_t data) 
{
    struct spi_ioc_transfer data_transfer = {0};
    uint8_t data_buf[1];

    pthread_mutex_lock(&lock);

    if (spidev_fd <= 0) {
        fprintf(stderr, "%s: spidev not yet opened\n", __func__);
        pthread_mutex_unlock(&lock);
        return -1;
    }

    gpiod_line_set_value(gpio_dc, 1);

    data_buf[0] = data;
    data_transfer.tx_buf = (unsigned long) data_buf;
    data_transfer.len = (uint32_t) sizeof(data_buf);

    if (ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &data_transfer) < 0) {
        fprintf(stderr, "%s: could not send data-message.\n", __func__);
        pthread_mutex_unlock(&lock);
        return -1;
    }

    pthread_mutex_unlock(&lock);
    return 0;
}

#ifndef NUMARGS
#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#endif
#define write_command_with_data(x, ...) (ssd1322_write_command(x, NUMARGS(__VA_ARGS__), __VA_ARGS__))
#define write_command(x) (ssd1322_write_command(x, 0, 0))

static void* ssd1322_thread_run(void * p) 
{
    (void)p;

    static struct timespec ts = 
	{
        .tv_sec = 0,
        .tv_nsec = 16666666, // 60Hz
    };

    while( spidev_buffer )
	{
        if( display_dirty )
		{
            ssd1322_refresh();
            display_dirty = false;
        }

        // If this event happens right before ssd1322_refresh(),
        // there is quite a bit of flashing. Possibly from being
        // at a weird sync point with the hardware refresh.
        event_post(event_data_new(EVENT_SCREEN_REFRESH));

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
    }

    return NULL;
}

void ssd1322_init() 
{
    if( pthread_mutex_init(&lock, NULL) != 0 ){
        fprintf(stderr, "%s: pthread_mutex_init failed\n", __func__);
        return;
    }

    spidev_buffer = calloc(SPIDEV_BUFFER_LEN, 1);
    if( spidev_buffer == NULL ){
        fprintf(stderr, "%s: couldn't allocate spidev_buffer\n", __func__);
        return;
    }

    spidev_fd = open_spi();
    if( spidev_fd < 0 )
	{
        fprintf(stderr, "%s: couldn't open %s.\n", __func__, SPIDEV_0_0_PATH);
        return;
    }

    gpio_0 = gpiod_chip_open_by_name(SSD1322_DC_AND_RESET_GPIO_CHIP);
    gpio_dc = gpiod_chip_get_line(gpio_0, SSD1322_DC_GPIO_LINE);
    gpio_reset = gpiod_chip_get_line(gpio_0, SSD1322_RESET_GPIO_LINE);

    // nomi ottenibili col comando gpioinfo
    gpiod_line_request_output(gpio_dc, "D/C", 0);
    gpiod_line_request_output(gpio_reset, "RST", 0);

    // Reset the display
    gpiod_line_set_value(gpio_reset, 1);
    usleep(100000); // 100 ms
    gpiod_line_set_value(gpio_reset, 0);
    usleep(100000); // 100 ms
    gpiod_line_set_value(gpio_reset, 1);
    usleep(100000); // 100 ms
  
    // Initialization sequence
    write_command(0x11);
    usleep(500000); 

    // MADCTL parameters (page 212)
    // Bits 0-1: Unused
    // Bit 2: 0=LCD refresh left-to-right, 1=right-to-left
    // Bit 3: 0=RGB mode, 1=BGR mode
    // Bit 4: 0=LCD refresh top-to-bottom, 1=bottom-to-top
    // Bit 5: Page/column order. 0=normal mode, 1=reverse mode
    // Bit 6: Column address order. 0=left-to-right, 1=right-to-left
    // Bit 7: Page address order. 0=top-to-bottom, 1=bottom-to-top
 //dbg   write_command_with_data(0x36, ST7789_Landscape180);// Rotate the screen by 270 degrees

	// RGB 16-bit color mode
    write_command_with_data(0x3a, 0x55);

    write_command_with_data(0xB2, 0x0c, 0x0c, 0, 0x33, 0x33);
    write_command_with_data(0xB7, 0x35);
	write_command_with_data(0xbb,0x28);
	write_command_with_data(0xc0,0x3c);
	write_command_with_data(0xc2,0x01);
	write_command_with_data(0xc3,0x0b);
	write_command_with_data(0xc4,0x20);
	write_command_with_data(0xc6,0x0f);
    write_command_with_data(0xd0, 0xa4, 0xa1);

	write_command_with_data(0xe0,0xd0,0x01,0x08,0x0f,0x11,0x2a,0x36,0x55,0x44,0x3a,0x0b,0x06,0x11, 0x20);
	write_command_with_data(0xe1,0xd0,0x02,0x07,0x0a,0x0b,0x18,0x34,0x43,0x4a,0x2b,0x1b,0x1c,0x22,0x1f);
	write_command_with_data(0x55,0xB0);

	// caset / raset
    write_command_with_data(0x2a, 0, 0, (ST7789_WIDTH - 1) >> 8, (ST7789_WIDTH - 1) & 0xFF  );
    write_command_with_data(0x2b, 0, 0, (ST7789_HEIGHT - 1) >> 8, (ST7789_HEIGHT - 1) & 0xFF  );

    write_command(0x29);
    usleep(100000); 

    /*
     *    // Flips the screen orientation if the device is a norns shield.
     *    if( platform() != PLATFORM_CM3 ){
     *        write_command_with_data(SSD1322_SET_DUAL_COMM_LINE_MODE, 0x04, 0x11);
}
else{
    write_command_with_data(SSD1322_SET_DUAL_COMM_LINE_MODE, 0x16, 0x11);
}
*/

    // Do not turn display on until the first update has been called,
    // otherwise previous GDDRAM (or noise) will display before the
    // "hello" startup screen.

    // Set high thread priority to avoid flashing.
    static struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_OTHER);

    // Start thread.
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&ssd1322_pthread_t, &attr, &ssd1322_thread_run, NULL);
    pthread_attr_destroy(&attr);
}

void ssd1322_deinit()
{
    if( spidev_fd > 0 )
	{
        // Drive RST low to turn off screen.
        gpiod_line_set_value(gpio_reset, 0);

        // Destroy file descriptors and handles.
        pthread_mutex_destroy(&lock);
        gpiod_line_release(gpio_reset);
        gpiod_line_release(gpio_dc);
        gpiod_chip_close(gpio_0);
        close(spidev_fd);

        if(spidev_buffer != NULL)
        {
            free(spidev_buffer);
            spidev_buffer = NULL;
        }
    }
}

void ssd1322_update(cairo_surface_t * surface_pointer, bool surface_may_have_color)
{
    int width = cairo_image_surface_get_width(surface_pointer);
    int height = cairo_image_surface_get_height(surface_pointer);
    if (width != ST7789_WIDTH || height != ST7789_HEIGHT) {
        fprintf(stderr, "Invalid surface dimensions\n");
        return;
    }

    pthread_mutex_lock(&lock);

    if( spidev_buffer != NULL && surface_pointer != NULL )
    {
        #ifdef DEBUG_DISPLAY
            int f = 33; //rand() % 255;
            surface_may_have_color=!surface_may_have_color; 
            for( uint32_t i = 0; i < SPIDEV_BUFFER_LEN; i ++)
                *(spidev_buffer + i)=f;
        #else
        const uint8_t *data = cairo_image_surface_get_data(surface_pointer);
        if( surface_may_have_color )
        {
            // Preserve luminance of RGB when converting to grayscale. Use the
            // closest multiple of 16 to the fraction to scale the channels'
            // grayscale value. Use a multiple of 16 because a 4-bit grayscale
            // value should fit into the upper nibble of the 8-bit value. The
            // decimal approximation is out of 256: 80 + 160 + 16 = 256.
            for( uint32_t i = 0; i < SPIDEV_BUFFER_LEN; i++)
            {
                const uint8x8x4_t pixel = vld4_u8(data + i);
                const uint16x8_t r = vmull_u8(pixel.val[2], vdup_n_u8( 64)); // R * ~ 0.2627
                const uint16x8_t g = vmull_u8(pixel.val[1], vdup_n_u8(160)); // G * ~ 0.6780
                const uint16x8_t b = vmull_u8(pixel.val[0], vdup_n_u8( 32)); // B * ~ 0.0593
                const uint8x8_t conversion = vaddhn_u16(vaddq_u16(r, g), b);
                vst1_u8(spidev_buffer + i, vsri_n_u8(conversion, conversion, 4));
            }
        } else
        {
            // If the surface has only been drawn to, we can guarantee that RGB are
            // all equal values representing a grayscale value. So, we can take any
            // of those channels arbitrarily. Use the green channel just because.
            for( uint32_t i = 0; i < SPIDEV_BUFFER_LEN; /*i += sizeof(uint8x8_t)*/i++ )
            {
                // VLD4 loads 4 vectors from memory. It performs a 4-way de-interleave from memory to the vectors.
                const uint8x8x4_t ARGB = vld4_u8(data + i);
                vst1_u8(spidev_buffer + i, vsri_n_u8(ARGB.val[1], ARGB.val[1], 4));
            }
        }        
        #endif

        display_dirty = true;
    } else
    {
        fprintf(stderr, "%s: spidev_buffer (%p) surface_pointer (%p)\n", __func__, spidev_buffer, surface_pointer);
    }

    pthread_mutex_unlock(&lock);
}

void ssd1322_refresh()
{
    struct spi_ioc_transfer transfer = {0};
    if( spidev_fd <= 0 )
    {
        fprintf(stderr, "%s: spidev not yet opened.\n", __func__);
        return;
    }

	// caset / raset
    write_command_with_data(0x2a, 0, 0, (ST7789_WIDTH - 1) >> 8, (ST7789_WIDTH - 1) & 0xFF  );
    write_command_with_data(0x2b, 0, 0, (ST7789_HEIGHT - 1) >> 8, (ST7789_HEIGHT - 1) & 0xFF  );

    if( should_turn_on )
	{
        write_command(0x29);
        should_turn_on = 0;
    }
    write_command(0x2c);

    pthread_mutex_lock(&lock);
    gpiod_line_set_value(gpio_dc, 1);
    const uint32_t spidev_bufsize = 8192; // Max is defined in /boot/config.txt
    int bytes_transferred=0;
    while((SPIDEV_BUFFER_LEN - bytes_transferred) > 0)
    {
        transfer.tx_buf = (unsigned long) (spidev_buffer + bytes_transferred);
        transfer.len = MIN(spidev_bufsize, SPIDEV_BUFFER_LEN - bytes_transferred);
        bytes_transferred +=  transfer.len;
        if( ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &transfer) < 0 ){
            fprintf(stderr, "%s: SPI data transfer %d of %d failed.\n",__func__, bytes_transferred,SPIDEV_BUFFER_LEN);
            goto early_return;
        }

    }

early_return:
    pthread_mutex_unlock(&lock);
}

void ssd1322_set_brightness(uint8_t b)
{
    write_command_with_data(0x51, b);
}

void ssd1322_set_contrast(uint8_t c)
{
    c=c+0;
    //non implementato
}

void ssd1322_set_display_mode(ssd1322_display_mode_t mode_offset)
{
    switch(mode_offset)
    {
        case SSD1322_DISPLAY_MODE_ALL_OFF:
            write_command(0x28);
            break;

        case SSD1322_DISPLAY_MODE_ALL_ON:
            write_command(0x29);
            break;

        case SSD1322_DISPLAY_MODE_NORMAL:
            write_command(0x20);
            break;

        case SSD1322_DISPLAY_MODE_INVERT:
            write_command(0x21);
            break;
    }
}

void ssd1322_set_gamma(double g)
{
    /*
     *    // (SSD1322 rev 1.2, P 29/60)
     *    // Section 8.8, Gray Scale Decoder:
     *    // "Note: Both GS0 and GS1 have no 2nd pre-charge (phase 3)
     *    //        and current drive (phase 4), however GS1 has 1st
     *    //        pre-charge (phase 2)."
     *
     *    // According to the above note, GS0 and GS1 should effectively
     *    // be skipped in the gamma curve calculation, because GS1 is
     *    // like the starting point so it should have a value of 0.
     *    uint8_t gs[16] = {0};
     *    double max_grayscale = SSD1322_GRAYSCALE_MAX_VALUE;
     *    for (int level = 0; level <= 14; level++) {
     *        double pre_gamma = level / 14.0;
     *        double grayscale = round(pow(pre_gamma, g) * max_grayscale);
     *        double limit = (grayscale > max_grayscale) ? max_grayscale : grayscale;
     *        gs[level + 1] = (uint8_t) limit;
}

	write_command_with_data(
    SSD1322_SET_GRAYSCALE_TABLE, // GS0 is skipped.
    gs[0x1], gs[0x2], gs[0x3], gs[0x4], gs[0x5],
    gs[0x6], gs[0x7], gs[0x8], gs[0x9], gs[0xA],
    gs[0xB], gs[0xC], gs[0xD], gs[0xE], gs[0xF]
    );
    write_command(SSD1322_ENABLE_GRAYSCALE_TABLE);
    */
    uint8_t gamma_curve[15];
    double max_grayscale = 255.0;
    for (int level = 0; level < 15; level++) {
        double pre_gamma = level / 14.0;
        double grayscale = round(pow(pre_gamma, g) * max_grayscale);
        double limit = (grayscale > max_grayscale) ? max_grayscale : grayscale;
        gamma_curve[level] = (uint8_t) limit;
    }

    write_command(0x26); // Set Gamma command
    for (int i = 0; i < 15; i++) {
        write_data(gamma_curve[i]);
    }
}

void ssd1322_set_refresh_rate(uint8_t hz)
{
    hz=hz+0;
	// non implementato
}

uint8_t* ssd1322_resize_buffer(size_t size)
{
    spidev_buffer = realloc(spidev_buffer, size);
    return (uint8_t *)spidev_buffer;
}

#undef NUMARGS
#undef write_command
#undef write_command_with_data
