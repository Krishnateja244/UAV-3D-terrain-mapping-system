import struct
from ..models.args import DetectorArgs
from ..framework.communicator import CommunicatorFactory
from ..devices.openimu.uart_provider import Provider

class OpenIMU(object):
    '''
    IMU Device Detector
    '''
    def __init__(self, **kwargs):
        self.communication = 'uart'
        self.communicator = None
        self._build_options(**kwargs)
        self.imudevice = None

    def find(self, callback):
        '''find if there is a connected device'''
        print('start to find device')
        if self.communicator is None:
            self.communicator = CommunicatorFactory.create(
                self.communication, self.options)

        self.communicator.find_device(callback)

    def _build_options(self, **kwargs):
        self.options = DetectorArgs(**kwargs)

    def onfinddev(self, device):
        self.imudevice = device
        #self.imudevice.setup(None)
    
    def startup(self):
        self.find(self.onfinddev)

    def close(self):
        self.communicator.close()

    def getdata(self, datatype):
        readback = self.imudevice.read_untils_have_data(datatype)
        #print(readback)
        if datatype == ('z1'):
            timeraw = (readback[0:4]) #time in ms
            # print(timeraw)
            time_ms = struct.unpack('I', bytes(timeraw))[0]
            xaccelraw = (readback[4:8]) #xaccel
            #print(xaccelraw)
            xaccel = struct.unpack('f', bytes(xaccelraw))[0]
            #print(struct.unpack('f', bytes(xaccelraw)))
            yaccelraw = (readback[8:12]) #yaccel
            #print(*yaccelraw)
            yaccel = struct.unpack('f', bytes(yaccelraw))[0]
            zaccelraw = (readback[12:16]) #zaccel
            zaccel = struct.unpack('f', bytes(zaccelraw))[0]
            xrateraw = (readback[16:20]) #xrate
            xrate = struct.unpack('f', bytes(xrateraw))[0]
            #print(xrate)
            yrateraw = (readback[20:24]) #yrate
            yrate = struct.unpack('f', bytes(yrateraw))[0]
            zrateraw = (readback[24:28]) #zrate
            zrate = struct.unpack('f', bytes(zrateraw))[0]
            xmagraw = (readback[28:32]) #xrate
            xmag = struct.unpack('f', bytes(xmagraw))[0]
            #print(xmag)
            ymagraw = (readback[32:36]) #yrate
            ymag = struct.unpack('f', bytes(ymagraw))[0]
            zmagraw = (readback[36:40]) #zrate
            zmag = struct.unpack('f', bytes(zmagraw))[0]
            imudata =[time_ms, xaccel, yaccel, zaccel, xrate, yrate, zrate, xmag, ymag, zmag]
        if datatype == ('e2'):
            timeraw = (readback[0:4]) #time in ms
            #print(timeraw)
            time_ms = struct.unpack('I', bytes(timeraw))[0]
            #time_ms = struct.unpack('I', bytes(readback[0:4]))[0] #unin32
            time_s = struct.unpack('d', bytes(readback[4:12]))[0]  #double
            roll = struct.unpack('f', bytes(readback[12:16]))[0]
            pitch = struct.unpack('f', bytes(readback[16:20]))[0]
            heading = struct.unpack('f', bytes(readback[20:24]))[0]
            xaccelraw = (readback[24:28]) #xaccel
            xaccel = struct.unpack('f', bytes(xaccelraw))[0]
            yaccelraw = (readback[28:32]) #yaccel
            yaccel = struct.unpack('f', bytes(yaccelraw))[0]
            zaccelraw = (readback[32:36]) #zaccel
            zaccel = struct.unpack('f', bytes(zaccelraw))[0]
            xaccelbiasraw = (readback[28:32]) #xaccelbias
            xaccelbias = struct.unpack('f', bytes(xaccelbiasraw))[0]
            yaccelbiasraw = (readback[32:36]) #yaccelbias
            yaccelbias = struct.unpack('f', bytes(yaccelbiasraw))[0]
            zaccelbiasraw = (readback[36:40]) #zaccelbias
            zaccelbias = struct.unpack('f', bytes(zaccelbiasraw))[0]
            xrateraw = (readback[48:52]) #xrate
            xrate = struct.unpack('f', bytes(xrateraw))[0]
            yrateraw = (readback[52:56]) #yrate
            yrate = struct.unpack('f', bytes(yrateraw))[0]
            zrateraw = (readback[56:60]) #zrate
            zrate = struct.unpack('f', bytes(zrateraw))[0]
            xratebiasraw = (readback[60:64]) #xratebias
            xratebias = struct.unpack('f', bytes(xratebiasraw))[0]
            yratebiasraw = (readback[64:68]) #yratebias
            yratebias = struct.unpack('f', bytes(yratebiasraw))[0]
            zratebiasraw = (readback[68:72]) #zratebias
            zratebias = struct.unpack('f', bytes(zratebiasraw))[0]
            xmagraw = (readback[84:88]) #xrate
            xmag = struct.unpack('f', bytes(xmagraw))[0]
            ymagraw = (readback[88:92]) #yrate
            ymag = struct.unpack('f', bytes(ymagraw))[0]
            zmagraw = (readback[92:96]) #zrate
            zmag = struct.unpack('f', bytes(zmagraw))[0]
            latitude = struct.unpack('d', bytes(readback[96:104]))[0]
            longitude= struct.unpack('d', bytes(readback[104:112]))[0]
            altitude = struct.unpack('d', bytes(readback[112:120]))[0]
            #print(latitude,longitude,altitude)
            imudata =[time_ms,time_s,roll,pitch,heading, xaccel, yaccel, zaccel,xrate, yrate, zrate,xmag,ymag,zmag,latitude,longitude,altitude,xaccelbias,yaccelbias,zaccelbias,xratebias,yratebias,zratebias]
            #print(xaccelbias,yaccelbias,zaccelbias,xratebias,yratebias,zratebias)
        return imudata
