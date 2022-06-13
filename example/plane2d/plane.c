# include       <ode/ode.h>
# include       <drawstuff/drawstuff.h>

# define  DRAWSTUFF_TEXTURE_PATH "../textures"
# define        STAGE_SIZE      8.0


#include "./plane_capi.h"

extern void cb_sim_start();
extern void cb_sim_step(int pause);

void cb_start() {
  cb_sim_start();
}

void cb_step(int pause) {
  cb_sim_step(pause);
}
