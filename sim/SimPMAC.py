import sys
import threading
from time import sleep
import time
import logging
import SocketServer
import npyscreen

current_milli_time = lambda: int(round(time.time() * 1000))

class SimulatedPmacApp(npyscreen.NPSAppManaged):
    def __init__(self):
        super(SimulatedPmacApp, self).__init__()
        self.server = None
        self.simulator = None
        self.server_thread = None

    def create_pmac(self, port):
        # Create the simulator
        self.simulator = PMACSimulator()
        # Start the simulator thread
        self.simulator_thread = threading.Thread(target=self.update)
        self.simulator_thread.daemon = True
        self.simulator_thread.start()
        # Setup the server
        HOST, PORT = "localhost", int(port)
        self.server = PMACServer((HOST, PORT), MyTCPHandler, simulator=self.simulator)
        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        # Exit the server thread when the main thread terminates
        self.server_thread.daemon = True
        self.server_thread.start()

    def get_status(self):
        values = []
        values.append("Axis 1 : " + str(self.simulator.axes[1].readPosition()))
        values.append("Axis 2 : " + str(self.simulator.axes[2].readPosition()))
        values.append("Axis 3 : " + str(self.simulator.axes[3].readPosition()))
        values.append("Axis 4 : " + str(self.simulator.axes[4].readPosition()))
        values.append("Axis 5 : " + str(self.simulator.axes[5].readPosition()))
        values.append("Axis 6 : " + str(self.simulator.axes[6].readPosition()))
        values.append("Axis 7 : " + str(self.simulator.axes[7].readPosition()))
        values.append("Axis 8 : " + str(self.simulator.axes[8].readPosition()))
        return values

    def update(self):
        while self.simulator.getRunning():
            self.simulator.update()
            sleep(0.05)

    def onStart(self):
        self.keypress_timeout_default = 100
        self.registerForm("MAIN", IntroForm())
        self.registerForm("MAIN_MENU", MainMenu())


class IntroForm(npyscreen.Form):
    def create(self):
        self.name = "Simulated PMAC"
        self.add(npyscreen.TitleText, labelColor="LABELBOLD", name="Set the port number for the simulator", value="",
                 editable=False)
        self.port = self.add(npyscreen.TitleText, name="Port Number: ", value="1025")

    def afterEditing(self):
        self.parentApp.create_pmac(int(self.port.value))
        self.parentApp.setNextForm("MAIN_MENU")


class MainMenu(npyscreen.FormBaseNew):
    def create(self):
        self.keypress_timeout = 1
        self.name = "Simulated PMAC"
        self.t2 = self.add(npyscreen.BoxTitle, name="Main Menu:", relx=2, max_width=24)  # , max_height=20)
        self.t3 = self.add(npyscreen.BoxTitle, name="Current Status:", rely=2,
                           relx=26)  # , max_width=45, max_height=20)

        self.t2.values = ["Exit"]
        self.t2.when_value_edited = self.button

    def while_waiting(self):
        #self.parentApp.update()
        self.t3.values = self.parentApp.get_status()
        self.t3.display()

    def button(self):
        selected = self.t2.entry_widget.value
        if selected == 0:
            self.parentApp.setNextForm(None)
            self.parentApp.switchFormNow()


#def threaded_simulator(simulator):
#    running = True
#    while running == True:
#        running = simulator.getRunning()
#        simulator.update()
#        sleep(0.1)


class PMACServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    def __init__(self, server_address, RequestHandlerClass, bind_and_activate=True, simulator=False):
        self.simulator = simulator
        SocketServer.TCPServer.__init__(self, server_address, RequestHandlerClass, bind_and_activate=bind_and_activate)


class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        while 1:
            # self.request is the TCP socket connected to the client
            self.data = self.request.recv(1024)  # .strip()
            if not self.data:
                break
            # First 8 bytes are packet header
            header = self.data[0:8]
            message = self.data[8:]

            logging.debug("Request: %s", message)
            response = self.server.simulator.parse(message)
            logging.debug("Response: %s", response)
            self.request.sendall(response + '\6')


class PMACAxis():
    def __init__(self):
        self.ivars = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 10000, -10000, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 50, 50, 50, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.position = 0.0
        self.dmd_position = 0.0
        self.velocity = 20
        self.done = "1"

    def getStatus(self):
        #return "88000080040" + self.done
        return "88000001840" + self.done

    def writeIVar(self, no, value):
        self.ivars[no] = value

    def readIVar(self, no):
        return self.ivars[no]

    def readPosition(self):
        return self.position

    def move(self, position):
        self.dmd_position = position

    def update(self):
        # print "position: " + str(self.position)
        # print "dmd_position: " + str(self.dmd_position)

        # FUDGE Divide by 20 and *1000 as 0.1 second update
        self.velocity = self.ivars[22] * 50
        if self.position < (self.dmd_position - self.velocity):
            self.position = self.position + self.velocity
            self.done = "0"
        elif self.position > (self.dmd_position + self.velocity):
            self.position = self.position - self.velocity
            self.done = "0"
        else:
            self.position = self.dmd_position
            self.done = "1"

class CoordinateSystem():
    def __init__(self, controller):
        self.controller = controller
        self.in_position = 1
        self.running_scan = False
        self.time_at_last_point = 0.0
        self.delta_time = 0

    def run_program(self):
        logging.debug("Running motion program")
        self.in_position = 0
        self.controller.set_p_var(4001, 1)  # Set status to active
        self.controller.set_p_var(4005, 0)  # total points
        self.controller.set_p_var(4006, 0)  # current index
        self.controller.set_p_var(4007, 0)  # current buffer
        self.time_at_last_point = current_milli_time()
        buffer_memory_address = self.controller.get_p_var(4008)
        self.delta_time = self.controller.read_memory_address(buffer_memory_address)
        self.running_scan = True

    def update(self):
        # Check if we are running the trajectory scan
        if self.running_scan:
            # Read the current time
            current_time = current_milli_time()
            # Read the time for last point
            # Work out if we should be moving to the next point
            if current_time > (self.time_at_last_point + self.delta_time):
                # Get the point
                point_index = self.controller.get_p_var(4006)
                # increment the point
                point_index = point_index + 1
                # Check to see if we have crossed buffer
                if point_index == self.controller.get_p_var(4004):
                    point_index = 0
                    if self.controller.get_p_var(4007) == 0:
                        self.controller.set_p_var(4007, 1)
                    else:
                        self.controller.set_p_var(4007, 0)

                # Read the buffer A or B
                current_buffer = self.controller.get_p_var(4007)
                if current_buffer == 0:
                    # We are in buffer A
                    current_buffer_fill = 4011
                    current_buffer_address = 4008
                else:
                    # We are in buffer B
                    current_buffer_fill = 4012
                    current_buffer_address = 4009

                # Set the new point index
                self.controller.set_p_var(4006, point_index)
                # Increment the total points
                total_points = self.controller.get_p_var(4005)+1
                self.controller.set_p_var(4005, total_points)
                logging.debug("Total points: %d", total_points)
                logging.debug("Current fill level of A: %d", self.controller.get_p_var(4011))
                if point_index == self.controller.get_p_var(current_buffer_fill):
                    # Scan has finished, we caught up
                    self.running_scan = False
                    self.in_position = 1
                    self.controller.set_p_var(4001, 2)  # Set status to IDLE
                else:
                    # Work out delta time
                    buffer_memory_address = self.controller.get_p_var(current_buffer_address) + point_index
                    self.delta_time = self.controller.read_memory_address(buffer_memory_address)
                    self.time_at_last_point = current_time
                    for axis in range(1,9):
                        position_memory_address = buffer_memory_address + (1000*axis)
                        current_position = self.controller.axes[axis].readPosition()
                        new_position = self.controller.read_position(position_memory_address)
                        velocity = (new_position - current_position) / self.delta_time
                        if velocity < 0.0:
                            velocity = -1.0 * velocity
                        self.controller.axes[axis].writeIVar(22, velocity) 
                        self.controller.axes[axis].move(float(self.controller.read_position(position_memory_address)))

                    logging.debug("Reading trajectory time: %d", self.delta_time)

                # Work out the velocity
                # Set the move demands
                # Update the counters


    def get_status(self):
        logging.debug("Status requested: %d", self.in_position)
        if self.in_position == 1:
            return "000000020000000000"
        else:
            return "000005000010000000"

class PMACSimulator():
    def __init__(self):
        # print "init called"
        self.running = True
        self.ivars = {}
        self.pvars = [0] * 16000
        self.memory = [0] * 1000000
        self.caxis = 1
        self.ccs = 1
        self.inhash = False
        self.inival = False
        self.sval = ""
        self.lastMessage = ""
        self.lastResponse = ""
        self.axes = {1: PMACAxis(),
                     2: PMACAxis(),
                     3: PMACAxis(),
                     4: PMACAxis(),
                     5: PMACAxis(),
                     6: PMACAxis(),
                     7: PMACAxis(),
                     8: PMACAxis()}
        self.cs = {1: CoordinateSystem(self),
                   2: CoordinateSystem(self),
                   3: CoordinateSystem(self),
                   4: CoordinateSystem(self),
                   5: CoordinateSystem(self),
                   6: CoordinateSystem(self),
                   7: CoordinateSystem(self),
                   8: CoordinateSystem(self),
                   9: CoordinateSystem(self),
                   10: CoordinateSystem(self),
                   11: CoordinateSystem(self),
                   12: CoordinateSystem(self),
                   13: CoordinateSystem(self),
                   14: CoordinateSystem(self),
                   15: CoordinateSystem(self),
                   16: CoordinateSystem(self)}
        # print self.axes
        self.setup_trajectory_interface()

    def setup_trajectory_interface(self):
        self.pvars[4020] = 1.1
        # Number of points in a buffer
        self.pvars[4004] = 1000
        # Address of A and B buffers
        self.pvars[4008] = 0x10020
        self.pvars[4009] = 0x12730

    def update(self):
        # print "Updating simulator"
        for axis in range(1, 9):
            self.axes[axis].update()
        for csno in range(1, 16):
            self.cs[csno].update()

    def parse(self, message):
        try:
            self.lastMessage = message
            # split the message by whitespace
            resp = ""
            self.response = ""
            self.lastResponse = ""
            for word in message.split():
                word = word.upper()
                logging.debug("Word: %s", word)
                resp = ""
                # Check for a starting hash
                if '#' in word:
                    index = word.find('#')
                    # Search for the number
                    num = word[index:]
                    self.caxis = int(filter(str.isdigit, num))
                    logging.debug("Changing axis to %d", self.caxis)
                if 'WL:$' in word:
                    logging.debug(word)
                    self.parse_memory_write(word)
                if '&' in word:
                    index = word.find('&')
                    # Search for the number
                    num = word[index:index+2]
                    self.ccs = int(filter(str.isdigit, num))
                    logging.debug("Changing CS to %d", self.ccs)
                if '->' in word:
                    if self.caxis == 1:
                        resp = "A"
                    elif self.caxis == 2:
                        resp = "B"
                    elif self.caxis == 3:
                        resp = "C"
                    elif self.caxis == 4:
                        resp = "U"
                    elif self.caxis == 5:
                        resp = "V"
                    elif self.caxis == 6:
                        resp = "W"
                    elif self.caxis == 7:
                        resp = "X"
                    elif self.caxis == 8:
                        resp = "Y"
                if 'B' in word and 'R' in word:
                    # Request to execute motion program
                    index = word.find('B')
                    # Search for the number
                    num = word[index:]
                    prog_no = int(filter(str.isdigit, num))
                    self.cs[self.ccs].run_program()
                    self.pvars[4001] = 1
                    logging.debug("Execute motion program %d for CS %d", prog_no, self.ccs)
                if 'J=' in word:
                    index = word.find('J=')
                    # Search for the number
                    num = word[index + 2:]
                    # print "Pos demand: " + num
                    logging.debug("Pos demand #%d (J=) %f", self.caxis, float(num))
                    self.axes[self.caxis].move(float(num))
                if "?" in word:
                    resp = self.parse_status_request(word)
                if word == "%":
                    resp = "100"
                if 'LIST' in word:
                    return chr(0x07) + "\r"
                if 'VER' in word:
                    resp = "1.942"
                if 'F' in word:
                    resp = "0.0"
                if 'V' in word:
                    resp = "0.0"
                if 'Q' in word:
                    resp = "0"
                if 'M' in word:
                    resp = "0"
                if 'P' in word:
                    if '#' in word:
                        resp = str(self.axes[self.caxis].readPosition())
                    else:
                        index = word.find('P')
                        # Search for the number
                        num = word[index:]
                        index = word.find('=')
                        writing = False
                        if index > -1:
                            value = num[index + 1:]
                            num = num[:index + 1]
                            writing = True
                        pvar = int(filter(str.isdigit, num))
                        if writing:
                            logging.debug("Writing P[%d] = %s", pvar, value)
                            self.pvars[pvar] = float(value)
                        else:
                            resp = str(self.pvars[int(filter(str.isdigit, num))])
                        #resp = "0"
                if 'CID' in word:
                    resp = "603382"
                elif 'I' in word:
                    index = word.find('I')
                    # Search for the number
                    num = word[index:]
                    index = word.find('=')
                    writing = False
                    if index > -1:
                        value = num[index + 1:]
                        num = num[:index + 1]
                        writing = True
                    ivar = int(filter(str.isdigit, num))
                    if ivar > 99 and ivar < 801:
                        axno = int(ivar / 100)
                        # print "Axis no: " + str(axno)
                        varno = ivar - (100 * axno)
                        # print "Var no: " + str(varno)
                        if writing == True:
                            self.axes[axno].writeIVar(varno, float(value))
                        else:
                            value = self.axes[axno].readIVar(varno)
                            # print "Value: " + str(value)
                            resp = str(value)
                    else:
                        if not writing:
                            resp = "0"
                self.response += resp + "\r"
        except:
            resp = ""

        #self.response = self.response[:-1]
        #self.response += "\r"
        self.lastResponse = self.lastResponse + resp + " "

        return self.response

    def set_p_var(self, index, value):
        self.pvars[index] = value

    def get_p_var(self, index):
        return self.pvars[index]

    def read_memory_address(self, address):
        return self.memory[address]

    def read_position(self, address):
        return self.convertToDouble(self.memory[address])

    def parse_memory_write(self, word):
        address = 0
        # Split by commas
        for data in word.split(','):
            logging.debug("Data item: %s", data)
            if 'WL:$' in data:
                # This is the address item so parse it
                index = data.find('WL:$')
                # Search for the number
                num = data[index + 4:]
                address = int(num, 16)
                logging.debug("Address to write to: %d", address)
            else:
                # This will be a write, so place the number in memory
                index = data.find('$')
                # Search for the number
                num = data[index + 1:]
                self.memory[address] = int(num, 16)
                logging.debug("Value to write: %d", self.memory[address])
                address += 1

    def parse_status_request(self, word):
        if word == "???":
            resp = "000000000000"
        else:
            if '??' in word:
                resp = self.cs[self.ccs].get_status()
            else:
                if '?' in word:
                    resp = self.axes[self.caxis].getStatus()
        return resp

    def is_number(self, s):
        try:
            int(s)
            return True
        except ValueError:
            return False

    def getRunning(self):
        return self.running

    def setRunning(self, running):
        self.running = running

    def convertToDouble(self, value):
        exponent = value & 0xFFF
        exponent = exponent - 0x800
        mantissa = value >> 12

        for index in range(0, exponent):
            mantissa = mantissa / 2.0

        while mantissa > 1.0:
            mantissa = mantissa / 2.0
        mantissa = mantissa * 2.0
        if exponent >= 0:
            for index in range(0, exponent):
                mantissa = mantissa * 2.0
        else:
            for index in range(0, (exponent*-1)):
                mantissa = mantissa / 2.0

        logging.debug("Converted value: %f", mantissa)
        return mantissa


if __name__ == "__main__":
    #logging.basicConfig(filename="simulator.log", filemode='w', level=logging.DEBUG)
    logging.basicConfig(filename="simulator.log", filemode='w', level=logging.ERROR)
    app = SimulatedPmacApp()
    app.run()
