# UART-IQRF

HDLC UART driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.

Features
*   compatible with TR-72(52) modules
*   supported programming languages: C
*   supported operating systems: Linux

Library contais following folders
```
├── examples                      Examples of UART-IQRF lib usage
│   │  
│   └── uart_example_dpa          Example of DPA communication via UART
│
├── include                       Folder contains header files for UART-IQRF src
└── uart_iqrf                     Main part of the UART-IQRF lib source codes
```

Quick command line compilation on the target
```bash
git clone https://github.com/iqrfsdk/clibuart.git
cd clibuart
mkdir -p .build; cd .build; cmake ..; make -j4
```

See [wiki](https://github.com/MICRORISC/iqrfsdk/wiki) for more information.
