# UE4 4.27 Linux Build Fix — ispc Headers

## Problem
UE4 4.27 source build on Linux fails with:
  error: use of undeclared identifier 'ispc'

## Root Cause
Setup.sh does not download pre-compiled ispc headers.
The build system expects .ispc.generated.h files to exist
before compiling C++ modules that depend on them.

## Fix Applied
1. Download ispc v1.16.1 binary:
   wget https://github.com/ispc/ispc/releases/download/v1.16.1/ispc-v1.16.1-linux.tar.gz
   tar -xzf ispc-v1.16.1-linux.tar.gz
   sudo cp ispc-v1.16.1-linux/bin/ispc /usr/local/bin/
   mkdir -p ~/UnrealEngine/Engine/Extras/ThirdPartyNotUE/ispc/Linux
   cp ispc-v1.16.1-linux/bin/ispc ~/UnrealEngine/Engine/Extras/ThirdPartyNotUE/ispc/Linux/

2. Pre-generate all ispc headers before running make:
   find . -name "*.ispc" | grep -v ThirdParty | grep -v Extras > /tmp/ispc_files.txt
   Run generate_ispc_headers.sh with include paths:
     -I Engine/Source/Runtime/Core/Public/Math
     -I Engine/Source/Runtime/Core/Public

3. Then run: make -j8
