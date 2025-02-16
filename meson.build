project(
    'a2',
    'c',
    default_options: [
        'debug=false',
        'optimization=s',
        'c_std=c23',
        'warning_level=2',
    ],
)

sources = [
    'src/main.c',
    'src/i2c.c',
    'stm32/src/startup_stm32.c',
    'stm32/src/gpio.c',
]

shared_args = [
    '-mthumb',
    '-mcpu=cortex-m4',
]

c_args = [
    '-DSTM32F411xE',
    '-ffunction-sections',
    '-fdata-sections',
] + shared_args

inc_dirs = include_directories(
    'stm32/inc',
    'stm32/CMSIS/Include',
    'stm32/CMSIS/Device/ST/STM32F4xx/Include',
)

link_args = [
    '-Wl,--gc-sections',
    '-nostartfiles',
    '-L../stm32/lds',
    '-Tstm32f411re.lds',
] + shared_args

executable('a2',
    sources,
    c_args: c_args,
    include_directories: inc_dirs,
    link_args: link_args,
)
