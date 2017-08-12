#include <stdlib.h>

#define CHANNEL_EMG		16
#define EMG_DATA_SIZE	32

#define HEADER		'$'

typedef struct
{
	int16_t data[CHANNEL_EMG];
} emgData;


emgData emgBuf;

bool EmgDataUnmarshal(const char *data, int size)
{
	static char buf[EMG_DATA_SIZE];
	static bool dataBuffering;
	static int cnt;
	int16_t *p = (int16_t*)buf;

	int idx = 0;

	bool dataReady = false;	
	
	if( dataBuffering )
	{
		for(; cnt < EMG_DATA_SIZE &&  idx < size ; idx++, cnt++ )
			buf[ cnt ] = data[ idx ];

		if( cnt == EMG_DATA_SIZE )
		{
			for(int i = 0; i < CHANNEL_EMG ; i++ )
				emgBuf.data[i] = p[i];
		
			dataReady = true;
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
					emgBuf.data[i] = p[i];
		
				dataReady = true;
				dataBuffering = false;
			}
		}
	}

	return dataReady;
}
