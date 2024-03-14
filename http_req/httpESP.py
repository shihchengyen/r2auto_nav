import requests
import os

class HttpESP:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        
        # The API endpoint
        self.url = "http://" + ip_address + "/openDoor"
        # Adding header
        self.headers = {"Content-Type": "application/json"}
        # Adding a payload
        self.payload = {"action": "openDoor", "parameters": {"robotId": "35"}}
        
    def request(self):
        # A get request to the API
        post_response = requests.post(self.url, headers=self.headers, json=self.payload)
        
        # return without parsing the response
        return post_response
    
    def pingESP(self):
        # ping the ESP
        # returns true if the ESP is online
        # -c sets number of package
        # -s sets the size of the package, 1 smallest
        # -w sets the time out, 1 second
        # https://www.geeksforgeeks.org/ping-command-in-linux-with-examples/
        return os.system("ping -c 1 -s 1 -w 1 " + self.ip_address) == 0
