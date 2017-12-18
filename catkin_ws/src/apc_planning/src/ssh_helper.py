# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 11:28:14 2017

@author: fhogan
"""

import paramiko
import time
import re
import Crypto.Cipher.AES
import os, sys
from stat import *

orig_new = Crypto.Cipher.AES.new

def fixed_AES_new(key, *ls):
  if Crypto.Cipher.AES.MODE_CTR == ls[0]:
    ls = list(ls)
    ls[1] = ''
  return orig_new(key, *ls)

class ssh:
    client = None
    chan = None

    def __init__(self, computer_id = 'server'):
        if computer_id=='main':
            address = "192.168.0.15"
        elif computer_id=='server':
            address = "192.168.0.191"
        elif computer_id=='frank':
            address = "192.168.0.11"
        elif computer_id=='maria':
            address = "192.168.0.10"

        username = "mcube"
        password = "thecube"

        #print("Connecting to server.")
        self.client = paramiko.client.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.client.AutoAddPolicy())
        Crypto.Cipher.AES.new = fixed_AES_new
        self.client.connect(address, username=username, password=password, look_for_keys=False)

    def sendCommand(self, command):
        if(self.client):
            stdin, stdout, stderr = self.client.exec_command(command)
            b = stderr.readlines()
            if b:
                for i in b:
                    print(i)
            a = stdout.readlines()
            if a:
                for i in a:
                    print(i)

    def put(self, localpath, remotepath):
        sftp = self.client.open_sftp()
        sftp.put(localpath, remotepath)
        sftp.close()

    def get(self, remotepath, localpath):
        sftp = self.client.open_sftp()
        sftp.get(remotepath, localpath)
        sftp.close()

    def mkdir(self, remotepath, ignore_existing=False):
        sftp = self.client.open_sftp()
        ''' Augments mkdir by adding an option to not fail if the folder exists  '''
        try:
            sftp.mkdir(remotepath)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise
        sftp.close()

    def remove(self, remotepath):
        sftp = self.client.open_sftp()
        sftp.remove(remotepath)
        sftp.close()

    def rmdir(self, remotepath):
        sftp = self.client.open_sftp()
        sftp.rmdir(remotepath)
        sftp.close()

    def close(self):
        self.client.close()

    def put_dir(self, localpath, remotepath):
        '''
        Uploads the contents of the local directory to the target path. The
            target directory needs to exists. All subdirectories in local are
            created under target.
        '''
        self.mkdir(remotepath)
        for item in os.listdir(localpath):
            print(item)
            print(localpath)
            if os.path.isfile(os.path.join(localpath, item)):
                print(os.path.join(localpath, item))
                print('%s/%s' % (remotepath, item))
                self.put(os.path.join(localpath, item), '%s/%s' % (remotepath, item))
            else:
                self.mkdir('%s/%s' % (remotepath, item), ignore_existing=True)
                self.put_dir(os.path.join(localpath, item), '%s/%s' % (remotepath, item))

    def get_dir(self, remotepath, localpath):
        self.download_dir(remotepath, localpath)

    def download_dir(self, remote_dir, local_dir):
        sftp = self.client.open_sftp()
        os.path.exists(local_dir) or os.makedirs(local_dir)
        dir_items = sftp.listdir_attr(remote_dir)
        for item in dir_items:

            # assuming the local system is Windows and the remote system is Linux
            # os.path.join won't help here, so construct remote_path manually
            remote_path = remote_dir + '/' + item.filename
            local_path = os.path.join(local_dir, item.filename)
            if S_ISDIR(item.st_mode):
                # print ('remote_path', remote_path)
                # print ('local_path', local_path)
                try:
                    self.download_dir(remote_path, local_path)
                except:
                    print ('[ERROR] Could not copy copy {}'.format(remote_path))
            else:
                sftp.get(remote_path, local_path)

    def get_dir_list(self, remotepath):
        sftp = self.client.open_sftp()
        return sftp.listdir(path=remotepath)

    def remotely_execute_script(self, script_remotepath, param_list):
        command = 'python ' + script_remotepath
        for param in param_list:
            command += ' "' + str(param).replace(' ', '*').replace(',', '?') + '"'
        print(command)
        try:
            (stdin, stdout, stderr) = self.client.exec_command(command)
            for line in stdout.readlines():
                print(line)
        except Exception as e:
            print(e)


if __name__ == "__main__":
    Conn = ssh('main')
