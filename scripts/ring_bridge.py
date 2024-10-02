#!/usr/bin/env python

import os
import fcntl
import binascii
import rospy
from rpwc_msgs.srv import stateExec, stateExecRequest, stateExecResponse
from rpwc_msgs.msg import TaskExec
from std_msgs.msg import String, Bool

# Definisci il percorso del dispositivo HID
hid_device_path = '/dev/hidraw4'

class CustomNode:
    def __init__(self):
        rospy.init_node('bluetooth_hid_reader', anonymous=True)

        rospy.wait_for_service( '/setup1/state_exec')

        self.clientStartMobileBaseAvvicinamento = rospy.ServiceProxy('/setup1/state_exec', stateExec)

        self.clientStartMobileBaseAllontanamento = rospy.ServiceProxy('/setup1/state_exec', stateExec)

        try:
            # Apri il file del dispositivo HID
            hid_device = open(hid_device_path, 'rb')

            # Imposta la modalità non-bloccante
            fcntl.fcntl(hid_device, fcntl.F_SETFL, os.O_NONBLOCK)

            alto1 = "03000100"  # Codice di pressione del pulsante 1
            basso2 = "03000200"  # Codice di pressione del pulsante 2

            flag_alto1 = False
            flag_basso2 = False

            # Loop principale
            while not rospy.is_shutdown():
                try:
                    # Leggi i dati dal dispositivo HID
                    data = hid_device.read(8)
                    if data:
                        # Converti i dati esadecimali in una stringa leggibile
                        hex_string = binascii.hexlify(data).decode('utf-8')
                        # Prepara il messaggio di richiesta
                        req = stateExecRequest()
                        req.state.data = 0

                        task1 = TaskExec()
                        task1.mode.data = 1
                        task1.sequenceExecutionType.data = 3
                        task1.taskInLoop = Bool(False)

                        # Filtra solo i messaggi di pressione
                        if hex_string != "03000000":  # Ignora i messaggi di rilascio
                            rospy.loginfo(f"Rilevata pressione: {hex_string}")
                            
                            # Verifica quale pulsante è stato premuto
                            if hex_string == alto1:
                                rospy.loginfo(f"Hai premuto il pulsante in alto, AVVICINAMENTO: {hex_string}")
                                task1.TasksID = [String('Task_0_0')]
                                req.TasksExec = [task1]
                                if not flag_alto1:
                                    rospy.loginfo(f"Chiama: {req}")
                                    response = self.clientStartMobileBaseAvvicinamento(req)
                                    rospy.loginfo(f"Answer: {response}")
                                    flag_alto1 = True
                                else:
                                    rospy.loginfo("Hai già chiamato avvicinamento")



                            if hex_string == basso2:
                                rospy.loginfo(f"Hai premuto il pulsante in basso, ALLONTANAMENTO: {hex_string}")
                                task1.TasksID = [String('Task_1_0')]
                                req.TasksExec = [task1]
                                if flag_alto1 and not flag_basso2:
                                    rospy.loginfo(f"Chiama: {req}")
                                    response = self.clientStartMobileBaseAllontanamento(req)
                                    rospy.loginfo(f"Answer: {response}")
                                    flag_basso2 = True
                                else:
                                    if not flag_alto1 and not flag_basso2:
                                        rospy.loginfo("Non puoi chiamare allontanamento, prima ti devi avvicinare")
                                    else:    
                                        rospy.loginfo("Hai già chiamato allontanamento")

                            
                            if flag_alto1 and flag_basso2:
                                flag_basso2 = False
                                flag_alto1 = False



                except IOError:
                    pass  # Ignora gli errori di lettura non bloccanti (nessun dato disponibile)

        except IOError as e:
            rospy.logerr(f"Errore nell'aprire il dispositivo HID: {e}")
        finally:
            if hid_device:
                hid_device.close()


    def run_class(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():

            rate.sleep()


if __name__ == '__main__':
    try:
        custom_node = CustomNode()
        custom_node.run_class()
        

    except rospy.ROSInterruptException:
        pass


