# Lib Serial

## Table of Contents

1. [Description](#description)
2. [Dependencies](#dependencies)
3. [How to build](#how-to-build)
    1. [Compile libraries](#compile-libraries)
    2. [Compile applications](#compile-application)
    3. [Compile unit test](#compile-unit-test)
4. [Known issues](#known-issues)
    1. [Create temporary file](#create-temporary-file)

## Description <a name="description"></a>

This project is a library to manage serial port on Windows.

The project is hosted on Github. Clone and repository initialization is done with:

```(bash)
git clone git@github.com:scorbeau/lib-serial.git
cd lib-serial
git submodule update --init --recursive
```

## Dependencies <a name="dependencies"></a>

This project is compiled with:
 * [msys2](https://www.msys2.org/) with mingw64:
     * mingw-w64-x86_64-gcc
     * mingw-w64-x86_64-gtest
     * git

## How to build <a name="how-to-build"></a>

### Compile libraries <a name="compile-libraries"></a>
To compile project use command below:

```(bash)
make libs
```

Outputs are located to *build/<compilation_mode>/\<arch>/lib/*.

### Compile applications <a name="compile-application"></a>
To compile project use command below:

```(bash)
make apps
```

Outputs are located to *build/<compilation_mode>/\<arch>/bin/*.

### Compile unit test <a name="compile-unit-test"></a>

To compile unit test use command below:

```(bash)
make tests
```

Outputs are located to *build/<compilation_mode>/\<arch>/lib*.

### Compile for specific target <a name="specific-target-complation"></a>

To compile for specific target, define variable **TARGET** when invoke *make*:

```(bash)
make TARGET=mingw64
```

By default target is build for gcc host machine.

If you want to add board support add config_\<TARGET>.mk in *ressources/toolchain* 
directory.

## Known issues <a name="known-issues"></a>

### msys2

#### Create temporary file <a name="create-temporary-file"></a>

Cannot create temporary file in C:\Program Files\msys2\tmp\: Permission denied

Change TMP and TEMP shell variable with commands below:

```(bash)
export TMP=$HOME/tmp
export TEMP=$HOME/tmp
```
