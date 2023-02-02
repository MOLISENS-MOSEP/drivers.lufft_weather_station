from time import sleep, time
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


def process_many_channels(umb, channels, receiver_id=1):
    
    # Yield successive n-sized
    # chunks from l.
    def divide_chunks(l, n):
        # looping till length l
        for i in range(0, len(l), n):
            yield l[i:i + n]
            
    channels_junks = divide_chunks(channels, 20)
    
    values = []
    statues = []
    for chj in channels_junks:
        values_junks, statuses_junks = umb.onlineDataQueryMultiOneCall(chj, receiver_id=receiver_id)
        values.extend(values_junks)
        statues.extend(statuses_junks)

    print(values)
    print(statues)


def main():

    single_request = 113                        # single channel request
    channels = [
        100, 120, 140, 160,
        112, 113,
        200, 220, 240, 260,
        215,
        300, 320, 340, 360,
        305, 325, 345, 365,
        310,
        400, 420, 440, 460, 480,
        401,
        403,
        500, 520, 540, 580,
        501,
        805,
        806
    ]  # channels to request

    with WS_UMB(device='/dev/rs485_adapter_1', baudrate=19200) as umb:
        # query_one_channel(umb, single_request, receiver_id=6)
        # query_multiple_channels(umb, channels, receiver_id=6)
        # print(umb.send_request(6, 45, 16, int(21).to_bytes(2,'little')))
        time_start = time()
        process_many_channels(umb, channels, receiver_id=6)
        sleep(1)
        process_many_channels(umb, channels, receiver_id=6)
        sleep(1)
        process_many_channels(umb, channels, receiver_id=6)
        print(time() - time_start)


if __name__ == "__main__":
    main()
