# UART-IQRF

UART driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.

Implemented API functions are:
```
uart_iqrf_init 			Initialization of the IQRF UART library for communication with IQRF module.
uart_iqrf_write			Writes data to IQRF module
uart_iqrf_read 			Receives data from IQRF module
uart_iqrf_destroy 		Destroys IQRF UART library object and releases UART port

```

See [wiki](https://github.com/MICRORISC/iqrfsdk/wiki) for more information.
