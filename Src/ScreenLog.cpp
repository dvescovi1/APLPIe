#include <time.h>
#include <sys/time.h>

char * TimeStamp()
{
	static struct timeval last;
	static char buf[32];
	struct timeval now;

	struct tm tmp;

	gettimeofday(&now, NULL);

	if (now.tv_sec != last.tv_sec)
	{
		localtime_r(&now.tv_sec, &tmp);
		strftime(buf, sizeof(buf), "%F %T", &tmp);
		last.tv_sec = now.tv_sec;
	}

	return buf;
}