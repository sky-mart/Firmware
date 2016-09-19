#include <drivers/device/spi.h>
#include <stdlib.h>
#include <string.h>

#include "ws2812.h"

using namespace device;

#define LOW_BIT_CHAR 	0xf0
#define HIGH_BIT_CHAR 	0xfe

const char *ws2812_path = "/dev/ws2812_leds";

class WS2812 : public device::SPI
{
public:
	WS2812(const char *path, int spi_bus);

	virtual int		init();
	virtual ssize_t	write(file_t *filp, const char *buffer, size_t buflen);
};

WS2812::WS2812(const char *path, int spi_bus) :
	SPI("WS2812", path, spi_bus, SPIDEV_DISPLAY, SPIDEV_MODE3, 10500000)
{
}

int WS2812::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}
	return ret;
}

ssize_t WS2812::write(file_t *filp, const char *buffer, size_t buflen)
{
	// warnx("WS2812::write");
	// for (int i = 0; i < buflen; i++) {
	// 	printf("%d ", buffer[i]);
	// }
	// printf("\n");
	size_t reallen = buflen * 8;
	uint8_t *send = (uint8_t *)malloc(reallen * sizeof(uint8_t));
	uint8_t *recv = (uint8_t *)malloc(reallen * sizeof(uint8_t));

	for (int i = 0; i < buflen; i++) {
		for (int j = 0; j < 8; j++) {
			if (buffer[i] & (1 << (8-j))) {
				send[i*8 + j] = HIGH_BIT_CHAR;
			} else {
				send[i*8 + j] = LOW_BIT_CHAR;
			}
		}
	}

	transfer(send, recv, reallen);
	
	free(send);
	free(recv);
	return buflen;
}


/** driver 'main' command */
extern "C" { __EXPORT int ws2812_main(int argc, char *argv[]); }

namespace ws2812
{

WS2812 **g_dev_ptr;
WS2812	*g_dev_ext; // on external bus

void start();
void test(); 

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start()
{
	g_dev_ptr = &g_dev_ext;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

 	/* create the driver */
#ifdef PX4_SPI_BUS_EXT
		*g_dev_ptr = new WS2812(ws2812_path, PX4_SPI_BUS_EXT);
#else
 		errx(0, "External SPI not available");
#endif

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}
 	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete(*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "no device on this bus");
}

void test() 
{
	if (*g_dev_ptr == nullptr) {
		errx(0, "start device");
	} else {
		int fd = open(ws2812_path, O_WRONLY);
		const char s[] = {0xff, 0xff, 0xff}; //, 
				// 0x00, 0x00, 0x00, 
				// 0x00, 0x00, 0x00, 
				// 0x00, 0x00, 0x00, 
				// 0x00, 0x00, 0x00, 
				// 0x00, 0xff, 0x00,
				// 0x00, 0x00, 0x00, 
				// 0x00, 0x00, 0x00,  
				// 0x00, 0x00, 0x00,
				// 0x00, 0xff, 0x00
				// };
		write(fd, s, 3);
		close(fd);
	}
}

} // namespace

int
ws2812_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		ws2812::start();
	}

	if (!strcmp(argv[1], "test")) {
		ws2812::test();
	}
	return 0;
}