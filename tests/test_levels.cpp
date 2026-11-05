#include "audio_levels.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
int main(int argc, char** argv) {
    viz::BandAnalyzer a; a.reset(32000);
    if (argc > 1 && argv[1][0] == 's') {          // sine sweep test
        const float F[10] = {80,150,280,500,900,1600,2800,5000,8500,13000};
        for (int k = 0; k < 10; ++k) for (float amp : {0.5f, 0.05f}) {
            a.reset(32000);
            std::vector<int16_t> buf(2*1024);
            for (int i = 0; i < 1024; ++i) { int16_t v = (int16_t)(amp*32767*sinf(6.2831853f*F[k]*i/32000.f)); buf[2*i]=buf[2*i+1]=v; }
            a.feed(buf.data(), 1024);
            uint8_t o[10]; a.read(o); int best = 0; for (int j=1;j<10;++j) if (o[j]>o[best]) best=j;
            printf("%6.0f Hz amp %.2f -> peak band %d  [", F[k], amp, best); for (int j=0;j<10;++j) printf("%3d ", o[j]); printf("]\n");
        }
        return 0;
    }
    // stream of mono s16 from stdin -> per-block bands (stereo duplicated)
    int16_t s; std::vector<int16_t> buf; unsigned long blocks = 0; double sum[10] = {0}; int mx[10] = {0};
    while (fread(&s, 2, 1, stdin) == 1) {
        buf.push_back(s); buf.push_back(s);
        if (buf.size() == 2*256) {
            if (a.feed(buf.data(), 256)) { uint8_t o[10]; a.read(o); ++blocks; for (int j=0;j<10;++j){sum[j]+=o[j]; if(o[j]>mx[j])mx[j]=o[j];} if (blocks % 200 == 0) { for (int j=0;j<10;++j) printf("%3d ", o[j]); printf("\n"); } }
            buf.clear();
        }
    }
    printf("blocks=%lu\nmean: ", blocks); for (int j=0;j<10;++j) printf("%5.0f ", sum[j]/blocks);
    printf("\nmax : "); for (int j=0;j<10;++j) printf("%5d ", mx[j]); printf("\n");
}
