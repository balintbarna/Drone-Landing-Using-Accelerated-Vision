from bagpy import bagreader
import pandas as pd

b = bagreader('/home/balint/Downloads/bags/2021-05-10-19-17-36.bag')

def export_topic(topic):
# replace the topic name as per your need
    data = b.message_by_topic(topic)
    data
    csv = pd.read_csv(data)
    csv # prints laser data in the form of pandas dataframe

list = [
    '/state_machine/state',
    '/landing_pos_error/local_frame',
    '/mavros/setpoint_position/local',
    '/mavros/local_position/pose',
    '/mavros/state'
]

for topic in list:
    export_topic(topic)
