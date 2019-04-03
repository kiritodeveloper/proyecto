from os import environ

is_debug = "IN_DEBUG_MODE" in environ
disable_open_cv = "IN_DEBUG_MODE" in environ
