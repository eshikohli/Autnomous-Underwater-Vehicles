# Garmin

from decimal import Decimal

from ... import nmea

class BF(nmea.ProprietarySentence):
    sentence_types = {}

    def __new__(_cls, manufacturer, data):
        name = manufacturer + data[0]
        cls = _cls.sentence_types.get(name, _cls)
        return super(BF, cls).__new__(cls)

    def __init__(self, manufacturer, data):
        self.sentence_type = manufacturer + data[0]
        super(BF, self).__init__(manufacturer, data)


class MSC(BF):
    """ BLUEFIN Payload Mission Command
    """
    fields = (
        ("Timestamp", Decimal),
        ("Payload Mission Command", "mission_command")
        )