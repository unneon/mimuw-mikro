project(
    'mimuw_mikro',
    'c',
    default_options: [
        'debug=true',
        'optimization=0',
        'c_std=c23',
        'warning_level=2',
    ],
)

objcopy = find_program('objcopy')

targets = [
    'small_1',
    'small_2',
    'small_3',
    'big',
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

common = static_library('mimuw_mikro',
    [
        'src/i2c.c',
        'stm32/src/startup_stm32.c',
        'stm32/src/gpio.c',
    ],
    c_args: c_args,
    include_directories: inc_dirs,
    link_args: link_args,
)

foreach target : targets
    elf = executable(target + '.elf',
        'src/' + target + '.c',
        c_args: c_args,
        include_directories: inc_dirs,
        link_args: link_args,
        link_with: common,
    )

    custom_target(target,
        input: elf,
        output: target + '.bin',
        command: [objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
        build_by_default: true,
    )
endforeach
