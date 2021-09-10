import paramiko
import numpy as np
import time

def scp_file(pc, file_name="obj.npy"):
    transport = paramiko.Transport(('10.52.21.133', 22))
    transport.connect(username='lvjun', password='lvjun')
    sftp = paramiko.SFTPClient.from_transport(transport)
    t0 = time.time()
    #np.save("tmp.npy", pc)
    sftp.get('/home/lvjun/robot/obj.npy', "test.npy")
    print(time.time()-t0)
    transport.close()

scp_file(None)