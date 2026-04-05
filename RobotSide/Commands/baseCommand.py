
class BaseCommand():
    def __init__(self, name, requiredSubsystem):
        self.name = name
        self.requiredSubsystem = requiredSubsystem
        self.interruptedFlag = False

    def execute(self):
        pass

    def isFinished(self):
        if self.interruptedFlag:
            return True
        return True
    
    def end(self):
        pass

    def interrupted(self):
        self.interruptedFlag = True
