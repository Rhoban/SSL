#!/bin/bash

find src/ai/ -iname '*.h' -o -iname '*cpp' |xargs clang-format -i -style=file
find src/client/ -iname '*.h' -o -iname '*cpp' |xargs clang-format -i -style=file
find src/viewer/ -iname '*.h' -o -iname '*cpp' |xargs clang-format -i -style=file
