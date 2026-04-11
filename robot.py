import time


class Robot:
    def __init__(self):
        self.totalTime = 0

    def init(self):
        pass

    def periodic(self):
        start = time.time()



        end = time.time()
        self.totalTime += (end - start)
        