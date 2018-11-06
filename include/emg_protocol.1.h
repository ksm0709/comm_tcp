#include <stdlib.h>
#include <queue>

#define CHANNEL_EMG		16

#define HEADER_SIZE     8
#define TIMESTAMP_SIZE  4
#define EMG_DATA_SIZE	32

#define EMG_DATA_ADDR	HEADER_SIZE + TIMESTAMP_SIZE
#define PACKET_SIZE		HEADER_SIZE + TIMESTAMP_SIZE + EMG_DATA_SIZE

#define HEADER		0xC691199927021942

typedef struct
{
	uint64_t header;
	uint32_t timestamp;
	int16_t data[CHANNEL_EMG];
} emgData;

std::queue<emgData> emgQue;

bool EmgDataUnmarshal(const char *data, int size)
{
	static char buf[EMG_DATA_SIZE];
	static bool dataBuffering;
	static int cnt;
	int16_t *p = (int16_t*)buf;
	emgData *emg_p;

	int idx = 0;
	emgData emgBuf;
/*
	if( dataBuffering )
	{
		for(; cnt < EMG_DATA_SIZE &&  idx < size ; idx++, cnt++ )
			buf[ cnt ] = data[ EMG_DATA_ADDR + idx ];

		if( cnt == EMG_DATA_SIZE )
		{
			for(int i = 0; i < CHANNEL_EMG ; i++ )
				emgBuf.data[i] = p[i];

			if( emgQue.size() > 1000 )
				emgQue.pop();

			emgQue.push( emgBuf );

			dataBuffering = false;
		}
	}
*/

	while( true )
	{
		if( size - idx >= PACKET_SIZE )
			emg_p = (emgData*)(data + idx);
		else
			break;

		ROS_INFO("idx :%d",idx);

		if( emg_p->header == HEADER )
		{	
			for(int i = 0; i < CHANNEL_EMG ; i++ )
			{
				emgBuf.data[i] = emg_p->data[i];
			}
			
			if( emgQue.size() > 1000 )
				emgQue.pop();

			emgQue.push( emgBuf );

			idx += PACKET_SIZE;

			ROS_INFO("test");
			

/*
            dataBuffering = true; 

			for(cnt = 0 ; cnt < EMG_DATA_SIZE &&  idx < size ; idx++, cnt++ )
				buf[ cnt ] = data[ EMG_DATA_ADDR + idx ];	
			
			if( cnt == EMG_DATA_SIZE )
			{
				for(int i = 0; i < CHANNEL_EMG ; i++ )
				{
					emgBuf.data[i] = p[i];
				}

				if( emgQue.size() > 1000 )
					emgQue.pop();


				emgQue.push( emgBuf );
				
				dataBuffering = false;
			}
*/
		}
		else
			idx++;
	}

	return true;
}
