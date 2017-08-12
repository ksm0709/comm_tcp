#pragma once
#include <string>

#define GLOVE_DATA_SIZE		144 // byte
#define GLOVE_DATA_NUM		18	// double

using namespace std;

typedef struct
{
	double pip, mp, press;
} FINGER;

typedef union
{
	struct{ 
		FINGER finger[5];
		double roll;
		double pitch;
		double yaw;
	}s;

	double data[18];
} GLOVE_DATA;

double ntohd(unsigned long long value)
{
	char* v = (char*)&value;
	char buf[8];
	double* p = (double*)buf;

	for(int i = 0; i < 8; i++)
		buf[7-i] = v[i];

	return *p;
}

string GloveDataMarshal(GLOVE_DATA* nData)
{
	string value;
	value.clear();
	value.append("$");
	value.append((const char *)nData, GLOVE_DATA_SIZE);

	return value;
}

GLOVE_DATA* GloveDataUnmarshal(char* nData, int size)
{	
	static int cnt;
	static GLOVE_DATA gloveRcvBuf;
	static bool dataBuffering;
	static char rcvBuf[GLOVE_DATA_SIZE];
	unsigned long long* pBuf = (unsigned long long*)rcvBuf;
	double *pdBuf = (double*)pBuf;
	int i;
	int idx = 0;

	GLOVE_DATA* ret = NULL;

	if( dataBuffering )
	{
		for (; cnt < GLOVE_DATA_SIZE && idx < size; cnt++, idx++)
			rcvBuf[cnt] = nData[idx];

		if (cnt == GLOVE_DATA_SIZE)
		{
			for (i = 0; i < GLOVE_DATA_NUM; i++)
				gloveRcvBuf.data[i] = pdBuf[i];

			dataBuffering = false;
			ret = &gloveRcvBuf;
		}
	}

	while (idx < size)
	{
		if (nData[idx] == '$')
		{
			idx++;
			dataBuffering = true;

			for (cnt = 0; cnt < GLOVE_DATA_SIZE && idx < size; cnt++, idx++)
				rcvBuf[cnt] = nData[idx];

			if (cnt == GLOVE_DATA_SIZE)
			{
				for (i = 0; i < GLOVE_DATA_NUM; i++)
					gloveRcvBuf.data[i] = pdBuf[i];

				dataBuffering = false;
				ret = &gloveRcvBuf;
			}
		}
	}

	return ret;
}
