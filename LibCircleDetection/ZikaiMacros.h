#pragma once

#ifndef MAX_LUT_SIZE 
#define MAX_LUT_SIZE 1024
#endif

#ifndef ZIKAI_TRUE
#define ZIKAI_TRUE 1
#endif /* !ZIKAI_TRUE */

/** ln(10) */
#ifndef ZIKAI_M_LN10
#define ZIKAI_M_LN10 2.30258509299404568402
#endif /* !ZIKAI_M_LN10 */

/** PI */
#ifndef ZIKAI_M_PI
#define ZIKAI_M_PI   3.14159265358979323846
#endif /* !ZIKAI_M_PI */

#ifndef ZIKAI_RELATIVE_ERROR_FACTOR
#define ZIKAI_RELATIVE_ERROR_FACTOR 100.0
#endif

/// Special defines
#ifndef ZIKAI_EDGE_VERTICAL
#define ZIKAI_EDGE_VERTICAL   1
#endif 

#ifndef ZIKAI_EDGE_HORIZONTAL
#define ZIKAI_EDGE_HORIZONTAL 2
#endif // !ZIKAI_EDGE_HORIZONTAL


#ifndef ZIKAI_ANCHOR_PIXEL 
#define ZIKAI_ANCHOR_PIXEL  254
#endif
#ifndef ZIKAI_EDGE_PIXEL
#define ZIKAI_EDGE_PIXEL    255
#endif


#ifndef ZIKAI_LEFT 
#define ZIKAI_LEFT  1
#endif
#ifndef ZIKAI_RIGHT 
#define ZIKAI_RIGHT 2
#endif
#ifndef ZIKAI_UP    
#define ZIKAI_UP    3
#endif
#ifndef ZIKAI_DOWN  
#define ZIKAI_DOWN  4
#endif


#ifndef ZIKAI_MAX_GRAD_VALUE
#define ZIKAI_MAX_GRAD_VALUE 128*256
#endif
#ifndef ZIKAI_EPSILON 
#define ZIKAI_EPSILON 1.0
#endif

#ifndef ZIKAI_LUT_SIZE
#define ZIKAI_LUT_SIZE (1024*4096)
#endif


// Special defines
#ifndef ZIKAI_EDGE_VERTICAL   
#define ZIKAI_EDGE_VERTICAL   1
#endif
#ifndef ZIKAI_EDGE_HORIZONTAL 
#define ZIKAI_EDGE_HORIZONTAL 2
#endif
#ifndef ZIKAI_EDGE_45         
#define ZIKAI_EDGE_45         3
#endif
#ifndef ZIKAI_EDGE_135        
#define ZIKAI_EDGE_135        4
#endif

#ifndef ZIKAI_MAX_GRAD_VALUE 
#define ZIKAI_MAX_GRAD_VALUE 128*256
#endif
#ifndef ZIKAI_EPSILON 
#define ZIKAI_EPSILON 1.0
#endif
#ifndef ZIKAI_MIN_PATH_LEN
#define ZIKAI_MIN_PATH_LEN 10
#endif