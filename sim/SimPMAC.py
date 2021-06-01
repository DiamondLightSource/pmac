import sys
import threading
from time import sleep
import time
import logging
import SocketServer
import npyscreen

current_milli_time = lambda: int(round(time.time() * 1000))

# Trajectory scanning M variable definitions
M_TRAJ_STATUS = 4034
M_TRAJ_VERSION = 4049
M_TRAJ_BUFSIZE = 4037
M_TRAJ_A_ADR = 4041
M_TRAJ_B_ADR = 4042
M_TRAJ_TOTAL_PTS = 4038
M_TRAJ_C_INDEX = 4039
M_TRAJ_C_BUF = 4040
M_TRAJ_BUF_FILL_A = 4044
M_TRAJ_BUF_FILL_B = 4045

class SimulatedPmacAppGui(npyscreen.NPSAppManaged):
    def __init__(self):
        super(SimulatedPmacAppGui, self).__init__()
        self.pmac_thread = PmacThread()

    def get_status(self):
        values = []
        axes = self.pmac_thread.simulator.axes
        values.append("Axis 1 : " + str(axes[1].readPosition()))
        values.append("Axis 2 : " + str(axes[2].readPosition()))
        values.append("Axis 3 : " + str(axes[3].readPosition()))
        values.append("Axis 4 : " + str(axes[4].readPosition()))
        values.append("Axis 5 : " + str(axes[5].readPosition()))
        values.append("Axis 6 : " + str(axes[6].readPosition()))
        values.append("Axis 7 : " + str(axes[7].readPosition()))
        values.append("Axis 8 : " + str(axes[8].readPosition()))
        return values

    def create_pmac(self, port):
        self.pmac_thread.create_pmac(port)

    def onStart(self):
        self.keypress_timeout_default = 100
        self.registerForm("MAIN", IntroForm())
        self.registerForm("MAIN_MENU", MainMenu())

class SimulatedPmacAppNoGui():
    def __init__(self, tcp_port):
        self.port = tcp_port
        self.pmac_thread = PmacThread()

    def run(self):
        print 'launching headless pmac simulator on port', self.port
        self.pmac_thread.create_pmac(self.port)
        while True:
            threading._sleep(.1)


class PmacThread():
    def __init__(self):
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

    def update(self):
        while self.simulator.getRunning():
            self.simulator.update()
            sleep(0.001)


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
            logging.debug("Response: %s", response.replace('\r', '\\r'))
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

    def stop(self):
        self.dmd_position = self.position

    def update(self):
        # print "position: " + str(self.position)
        # print "dmd_position: " + str(self.dmd_position)

        self.velocity = self.ivars[22]
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
        self.controller.set_m_var(M_TRAJ_STATUS, 1)  # Set status to active
        self.controller.set_m_var(M_TRAJ_TOTAL_PTS, 0)  # total points
        self.controller.set_m_var(M_TRAJ_C_INDEX, 0)  # current index
        self.controller.set_m_var(M_TRAJ_C_BUF, 0)  # current buffer
        self.time_at_last_point = current_milli_time()
        buffer_memory_address = self.controller.get_m_var(M_TRAJ_A_ADR)
        self.delta_time = (self.controller.read_memory_address(buffer_memory_address)&0xFFFFFF) / 1000
        logging.debug("self.delta_time: %f", self.delta_time)
        for axis in range(1, 9):
            position_memory_address = buffer_memory_address + (1000 * axis)
            current_position = self.controller.axes[axis].readPosition()
            new_position = self.controller.read_position(position_memory_address)
            if self.delta_time == 0:
                logging.debug("Zero delta time for CS")
                velocity = 1.0
            else:
                velocity = (new_position - current_position) / self.delta_time
            if velocity < 0.0:
                velocity *= -1.0
            self.controller.axes[axis].writeIVar(22, velocity)
            self.controller.axes[axis].move(float(new_position))
        self.running_scan = True

    def update(self):
        # Check if we are running the trajectory scan
        if self.running_scan:
            # Check for an abort
            status = self.controller.get_m_var(M_TRAJ_STATUS)  # Set status to active
            if status != 1:
                # Aborted to stop the scan
                self.running_scan = False
                self.in_position = 1
            else:
                # Read the current time
                current_time = current_milli_time()
                # Read the time for last point
                # Work out if we should be moving to the next point
                if current_time > (self.time_at_last_point + self.delta_time):
                    # Get the point
                    point_index = self.controller.get_m_var(M_TRAJ_C_INDEX)
                    # increment the point
                    point_index = point_index + 1
                    # Check to see if we have crossed buffer
                    if point_index == self.controller.get_m_var(M_TRAJ_BUFSIZE):
                        point_index = 0
                        if self.controller.get_m_var(M_TRAJ_C_BUF) == 0:
                            self.controller.set_m_var(M_TRAJ_C_BUF, 1)
                            self.controller.set_m_var(M_TRAJ_BUF_FILL_A, 0)
                        else:
                            self.controller.set_m_var(M_TRAJ_C_BUF, 0)
                            self.controller.set_m_var(M_TRAJ_BUF_FILL_B, 0)

                    # Read the buffer A or B
                    current_buffer = self.controller.get_m_var(M_TRAJ_C_BUF)
                    if current_buffer == 0:
                        # We are in buffer A
                        current_buffer_fill = M_TRAJ_BUF_FILL_A
                        current_buffer_address = M_TRAJ_A_ADR
                    else:
                        # We are in buffer B
                        current_buffer_fill = M_TRAJ_BUF_FILL_B
                        current_buffer_address = M_TRAJ_B_ADR

                    # Set the new point index
                    self.controller.set_m_var(M_TRAJ_C_INDEX, point_index)
                    # Increment the total points
                    total_points = self.controller.get_m_var(M_TRAJ_TOTAL_PTS)+1
                    self.controller.set_m_var(M_TRAJ_TOTAL_PTS, total_points)
                    logging.debug("Total points: %d", total_points)
                    if point_index == self.controller.get_m_var(current_buffer_fill):
                        # Scan has finished, we caught up
                        self.running_scan = False
                        self.in_position = 1
                        self.controller.set_m_var(M_TRAJ_STATUS, 2)  # Set status to IDLE
                    else:
                        # Work out delta time
                        buffer_memory_address = self.controller.get_m_var(current_buffer_address) + point_index
                        self.delta_time = (self.controller.read_memory_address(buffer_memory_address)&0xFFFFFF) / 1000
                        self.time_at_last_point = current_time
                        for axis in range(1,9):
                            position_memory_address = buffer_memory_address + (1000*axis)
                            current_position = self.controller.axes[axis].readPosition()
                            new_position = self.controller.read_position(position_memory_address)
                            if self.delta_time == 0:
                                logging.debug("Zero delta time for CS")
                                velocity = 1.0
                            else:
                                velocity = (new_position - current_position) / self.delta_time
                            if velocity < 0.0:
                                velocity *= -1.0
                            self.controller.axes[axis].writeIVar(22, velocity)
                            self.controller.axes[axis].move(float(new_position))

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
        self.mvars = [0] * 16000
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
        self.setup_some_standard_mvars()

    def setup_some_standard_mvars(self):
        # pmac interupt timings (for processor timing calcs)
        self.mvars[70] = 11990
        self.mvars[71] = 554
        self.mvars[72] = 2621
        self.mvars[73] = 76

    def setup_trajectory_interface(self):
        self.mvars[M_TRAJ_VERSION] = 3.0
        # Number of points in a buffer
        self.mvars[M_TRAJ_BUFSIZE] = 1000
        # Address of A and B buffers
        #self.mvars[M_TRAJ_A_ADR] = 0x10020
        self.mvars[M_TRAJ_A_ADR] = 0x40000
        #self.mvars[M_TRAJ_B_ADR] = 0x12730
        self.mvars[M_TRAJ_B_ADR] = 0x30000

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
                if '/' in word:
                    self.axes[self.caxis].stop()
                if "?" in word:
                    resp = self.parse_status_request(word)
                if "%" in word:
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
                    index = word.find('M')
                    # Search for the number
                    num = word[index:]
                    index = word.find('=')
                    writing = False
                    if index > -1:
                        value = num[index + 1:]
                        num = num[:index + 1]
                        writing = True
                    mvar = int(filter(str.isdigit, num))
                    if writing:

                        logging.debug("Writing M[%d] = %s", mvar, value)
                        self.mvars[mvar] = float(value)
                    else:
                        resp = str(self.mvars[int(filter(str.isdigit, num))])
                if 'CPU' in word:
                    resp = "DSP56321"
                elif 'P' in word:
                    if '#' in word or word.strip() == 'P':
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
                    if ivar > 99 and ivar < 901:
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

    def set_m_var(self, index, value):
        self.mvars[index] = value

    def get_m_var(self, index):
        return self.mvars[index]

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
        logging.debug("Converting value: %x", value)
        exponent = value & 0xFFF
        exponent = exponent - 0x800
        mantissa = value >> 12
        if mantissa & 0x800000000:
            negative = -1.0
            mantissa = 0xFFFFFFFFF - mantissa
        else:
            negative = 1.0

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

        mantissa *= negative
        logging.debug("Converted value: %f", mantissa)
        return mantissa


if __name__ == "__main__":
    # logging.basicConfig(filename="simulator.log", filemode='w',
    #                     level=logging.DEBUG)
    logging.basicConfig(filename="/tmp/pmac-simulator.log", filemode='w',
                        level=logging.ERROR)

    # a single command line parameter provides the port and runs with no UI
    # no parameter means run interactively
    if len(sys.argv) == 2:
        app = SimulatedPmacAppNoGui(sys.argv[1])
    else:
        app = SimulatedPmacAppGui()

    app.run()
