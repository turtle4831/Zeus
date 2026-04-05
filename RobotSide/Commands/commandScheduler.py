class CommandScheduler:
    def __init__(self):
        self.commands = []
        
    def schedule(self, command):
        self.commands.append(command)

    def loop(self):
        for command in self.commands:
            while not command.isFinished() or command.interrupted():
                command.execute()

                for other_command in self.commands:
                    if other_command != command and other_command.requiredSubsystem == command.requiredSubsystem:
                        command.interrupted()
                        command.end()

            command.end()
            self.commands.remove(command)

    def cancel(self, command):
        if command in self.commands:
            command.interrupted()
            self.commands.remove(command)

   