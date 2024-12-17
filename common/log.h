#include <stdio.h>

/*红底黑字*/
#define LOG_ERROR(...)																							\
	printf("\033[41;37mERROR(%s:%d)\033[0m: ", __FILE__, __LINE__);		\
	printf(__VA_ARGS__);																							

/*黄底黑字*/
#define LOG_WARNING(...)																						\
	printf("\033[43;30mWARNING(%s:%d)\033[0m: ", __FILE__, __LINE__);	\
	printf(__VA_ARGS__);																							

/*绿底白字*/
#define LOG_DEBUG(...)									\
	printf("\033[42;37mDEBUG\033[0m: ");	\
	printf(__VA_ARGS__);									
