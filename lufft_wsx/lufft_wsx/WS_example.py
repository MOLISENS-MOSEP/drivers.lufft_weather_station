from time import sleep
from WS_UMB import WS_UMB

def query_one_channel(umb, channel, receiver_id=1):
    '''query one channel'''

    value, status = umb.onlineDataQuery(channel, receiver_id=receiver_id)
    if status != 0:
        print(umb.checkStatus(status))
    else:
        print(value)

def query_multiple_channels(umb, channels, receiver_id=1):
    '''Query multiple channels by providing list of channels'''

    values, statuses = umb.onlineDataQueryMulti(channels, receiver_id=receiver_id)
    print("per channel query list: " + str(values))

def query_multiple_channels_one_call(umb, channels, receiver_id=1):
    '''Query multiple channels in one call by proving list of channels'''

    values, statuses = umb.onlineDataQueryMultiOneCall(channels, receiver_id=receiver_id)
    print("one call query list:    " + str(values))

def main():

    single_request = 113                        # single channel request
    # channels = [113, 4630, 113, 113, 4630, 113] # channels to request

    with WS_UMB(device='COM8', baudrate=19200) as umb:
        query_one_channel(umb, single_request, receiver_id=6)
        # query_multiple_channels(umb, channels)
        # query_multiple_channels_one_call(umb, channels)

if __name__ == "__main__":
    main()
