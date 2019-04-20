#!/bin/bash

find src/ai/ -iname '*.h' -o -iname '*cpp' |xargs clang-format-6.0 -i -style=file
find src/client/ -iname '*.h' -o -iname '*cpp' |xargs clang-format-6.0 -i -style=file
find src/viewer/ -iname '*.h' -o -iname '*cpp' |xargs clang-format-6.0 -i -style=file
