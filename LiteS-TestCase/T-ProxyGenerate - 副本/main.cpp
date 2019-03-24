#include "CEngine.h"
#include "CSIFT.h"


int main(){
	CSIFT detector;

	CImage* image = new CImage();
	image->ncols = 300;
	image->nrows = 200;
	image->data = new float[image->nrows*image->ncols];
	for (int i = 0; i < image->ncols*image->nrows; i++)
		image->data[i] = rand();
	detector.run(image);

    return 0;
}
