# RTC I2C Write Bug Repro Sample

This repository is a minimal example to reproduce an issue with the RTC I2C peripheral as originally
documented in [https://github.com/espressif/esp-idf/issues/11037](https://github.com/espressif/esp-idf/issues/11037).

Without modifying the IDF code, it appears the _first_ write to the I2C peripheral on the common ENS210 environment
sensor does not work. All additional writes appear to succeed without issue. It is always the first write that fails.
Ocassionally the first write will work but it appears to be random.

By default, this project uses VSCode Dev containers to build an image with the 5.1.1 version of IDF
WITHOUT the fix applied. Building and flashing this project will produce the following error output.

```
I (315) main_task: Calling app_main()
I (315) main: Initializing RTC I2C ... 
I (325) main: ok
I (325) main: Initializing ENS210 sensor ... 
I (335) main: Writing to SYS_CTRL register
E (345) ulp_riscv_i2c: Write Failed!
E (345) ulp_riscv_i2c: RTC I2C Interrupt Raw Reg 0x12c
E (345) ulp_riscv_i2c: RTC I2C Status Reg 0x66860000
I (405) main: Writing to SYS_CTRL register
I (465) main: Writing to SYS_CTRL register
I (525) main: Writing to SENS_RUN register
I (585) main: Writing to SYS_CTRL register to enable low power mode
I (645) main: Writing to SENS_START register
I (715) main: Status: 0x01

I (735) main: ENS210 Device ID: 0x0210

I (735) main: ok
I (935) main: Reading temperature and humidity data from ENS210 sensor ...
I (995) main: Temperature = 24.178 deg Celsius
I (995) main: Humidity = 25.898%
I (995) main_task: Returned from app_main()
```

Notice the first write fails but all subsequent writes seem to work fine.

## Applying the Recommended Fix
The fix is applied either by manually copying the file from ./devcontainer/idf_patches/ulp_riscv_i2c.c 
to it's appropriate place in the IDF folder structure(/idf/components/ulp/ulp_riscv/ulp_riscv_i2c.c) or 
by uncommenting line 86 in the Dockerfile of this project and rebuilding the dev container. 

Even with the "fix" applied the error still occurs at random. In the case of failure it is ALWAYS the first
write that fails. All subsequent writes work just fine.

## Expected Behavior

`ulp_riscv_i2c_master_write_to_device` should work without failure.