import os

# If it's true means that we are in debug mode (local execution)
# to accomplish that automatically we should declare the RASPBERRY_DEPLOY variable
# in the Robot or set is_debug to false there and true in local
is_debug = 'RASPBERRY_DEPLOY' not in os.environ
