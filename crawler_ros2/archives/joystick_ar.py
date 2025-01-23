
class Joystick():
    def __init__(self) -> None:
        
        self.pads = inputs.devices.gamepads

    def error(self):
        if len(self.pads) == 0:

            raise Exception("Couldn't find any Gamepads!")
        
    def status(self):
        events = inputs.get_gamepad()
        for event in events:

            # print(event.ev_type, event.code, event.state)
            # print(event.code)

            return [event.ev_type, event.code, event.state]