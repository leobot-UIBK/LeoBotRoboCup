#!/usr/bin/env python
# SiHa: Hacky Script to open/close Emika breaks
# There is no official way of opening the breaks other than the webinterface
# https://github.com/frankaemika/libfranka/issues/17
# prerequisite: python requests

import requests, rospy, sys

def lock_franka(emikaip, flag_lock, path_auth):
    #emikaip="192.168.1.1"
    print('un'*bool(flag_lock) + 'locking Franka.')
    headers = {
        'User-Agent': 'Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:89.0) Gecko/89.0 Firefox/89.0',
        'Accept': '*/*',
        'Accept-Language': 'en-GB,en;q=0.5',
        'Referer': 'https://'+emikaip+'/desk/',
        'content-type': 'application/x-www-form-urlencoded',
        'Origin': 'https://'+emikaip+'',
        'Connection': 'keep-alive',
    }
    try: 
        with open(path_auth, 'r') as file: 
            str_cookie = file.read().replace('\n', '')
    except: 
        rospy.logerr('Authentification cookie not found at the path from the launch file. Maybe it needs to be created? For more information read the RoboCup Wiki of the LeoBot. ')
    cookies = {
        'authorization': str_cookie}
        # 'eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzUxMiJ9.eyJuYW1lIjoiYWRtaW4iLCJyb2xlIjp7ImF1dGhvcml6YXRpb24iOlt7InBlcm1pc3Npb24iOiJSZWFkV3JpdGUiLCJyZXNvdXJjZSI6IlRhc2tzIn0seyJwZXJtaXNzaW9uIjoiUmVhZFdyaXRlIiwicmVzb3VyY2UiOiJTa2lsbHMifSx7InBlcm1pc3Npb24iOiJSZWFkV3JpdGUiLCJyZXNvdXJjZSI6IlBhcmFtZXRlcnMifSx7InBlcm1pc3Npb24iOiJSZWFkV3JpdGUiLCJyZXNvdXJjZSI6IkV4ZWN1dGlvbiJ9LHsicGVybWlzc2lvbiI6IlJlYWRXcml0ZSIsInJlc291cmNlIjoiU3RhdHVzIn0seyJwZXJtaXNzaW9uIjoiUmVhZFdyaXRlIiwicmVzb3VyY2UiOiJCdW5kbGVzIn0seyJwZXJtaXNzaW9uIjoiUmVhZFdyaXRlIiwicmVzb3VyY2UiOiJTY3JpcHRzIn0seyJwZXJtaXNzaW9uIjoiUmVhZFdyaXRlIiwicmVzb3VyY2UiOiJBZG1pbiJ9LHsicGVybWlzc2lvbiI6IlJlYWRXcml0ZSIsInJlc291cmNlIjoiU2FmZXR5In1dLCJuYW1lIjoiYWRtaW4ifX0.kh2uZBMwlfHjnoV8PwzT6c4aJj-z-9VFD_ZnPamsgU3MV-Q0yHSBNvCYaYLKjYfMihVGBKP5GngNzO-kA1tDFA'
    #}

    #Closing Braks
    if flag_lock == 'true': 
        response = requests.post('https://'+emikaip+'/desk/api/robot/close-brakes',headers=headers, cookies=cookies, verify=False)
        rospy.loginfo('Franka brakes closing.')
    #Opening Braks
    else: 
        response = requests.post('https://'+emikaip+'/desk/api/robot/open-brakes',headers=headers, cookies=cookies, verify=False)
        rospy.loginfo('Franka brakes opening.')

    #if bool(flag_lock) == False: 

def main(args):
    emikaip = args[1]
    flag_lock = args[2]
    path_auth = args[3]

    rospy.init_node('Franka_joint_locks', anonymous=False)
    rospy.loginfo("Authentificating franka with IP " + str(emikaip) + ", with cookie read from" + str(path_auth) + ".")
    lock_franka(emikaip, flag_lock, path_auth)                
    # rospy.sleep(1)

if __name__ == '__main__':
    main(sys.argv)


