from multiprocessing import Value


class SharedValue(object):
    def __init__(self, type_of_value, value):
        self.__shared_value = Value(type_of_value, value)

    def get(self):
        """
        Get the value of the shared value
        :return:
        """
        with self.__shared_value.get_lock():
            value = self.__shared_value.value
        return value

    def set(self, value):
        """
        Set the value of the shared value
        :param value:
        :return:
        """
        with self.__shared_value.get_lock():
            self.__shared_value.value = value
