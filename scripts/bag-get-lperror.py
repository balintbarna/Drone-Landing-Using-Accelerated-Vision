from bagpy import bagreader
import pandas as pd

b = bagreader('/home/balint/Downloads/bags/2021-05-10-19-17-36.bag')

# replace the topic name as per your need
data = b.message_by_topic('/landing_pos_error/local_frame')
data
csv = pd.read_csv(data)
csv # prints laser data in the form of pandas dataframe