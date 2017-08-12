#include <stdlib.h>
#include <queue>

#define CHANNEL_EMG		16
#define EMG_DATA_SIZE	32

#define HEADER		'$'

typedef struct
{
	int16_t data[CHANNEL_EMG];
} emgData;

std::queue<emgData> emgQue;

bool EmgDataUnmarshal(const char *data, int size)
{
	static char buf[EMG_DATA_SIZE];
	static bool dataBuffering;
	static int cnt;
	int16_t *p = (int16_t*)buf;

	int idx = 0;
	emgData emgBuf;

	if( dataBuffering )
	{
		for(; cnt < EMG_DATA_SIZE &&  idx < size ; idx++, cnt++ )
			buf[ cnt ] = data[ idx ];

		if( cnt == EMG_DATA_SIZE )
		{
			for(int i = 0; i < CHANNEL_EMG ; i++ )
				emgBuf.data[i] = p[i];

			if( emgQue.size() > 100 )
				emgQue.pop();

			emgQue.push( emgBuf );

			dataBuffering = false;
		}
	}

	while( idx < size )
	{
		if( data[ idx++ ] == HEADER )
		{
			dataBuffering = true; 

			for(cnt = 0 ; cnt < EMG_DATA_SIZE &&  idx < size ; idx++, cnt++ )
				buf[ cnt ] = data[ idx ];	
			
			if( cnt == EMG_DATA_SIZE )
			{
				for(int i = 0; i < CHANNEL_EMG ; i++ )
				{
					emgBuf.data[i] = p[i];
				}

				if( emgQue.size() > 100 )
					emgQue.pop();


				emgQue.push( emgBuf );
				
				dataBuffering = false;
			}
		}
	}

	return true;
}
