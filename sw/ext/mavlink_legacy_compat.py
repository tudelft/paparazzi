#!/usr/bin/env python3

import runpy
import sys
import types


def install_future_compatibility():
    future = types.ModuleType("future")
    standard_library = types.ModuleType("future.standard_library")
    standard_library.install_aliases = lambda: None
    utils = types.ModuleType("future.utils")
    utils.iteritems = dict.items

    future.standard_library = standard_library
    future.utils = utils
    sys.modules["future"] = future
    sys.modules["future.standard_library"] = standard_library
    sys.modules["future.utils"] = utils


if __name__ == "__main__":
    generator = sys.argv.pop(1)
    sys.argv[0] = generator
    install_future_compatibility()
    runpy.run_path(generator, run_name="__main__")