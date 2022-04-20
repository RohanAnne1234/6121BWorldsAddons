/*
Credit to VRC Team 315G for most of this code
*/

#include <stdio.h>
#include "main.h"

int autonNumber;
int driverState;

static const char *btnm_map[] = {"R_Both", "L_Both", "R_Neuts", "L_Neuts", "\n",
								 "AWP", "R_Rush", "L_Rush", "R_MidRush", "\n",
								 "DriverSkills", "ProgSkills", ""};
static const char *auton_strings[] = {"R.Both", "L.Both", "R.Neuts", "L.Neuts", "AWP", "R.Rsh", "L.Rsh", "R.MidRush", "DrivSkills", "ProgSkills"};



typedef  FILE * pc_file_t;

static lv_fs_res_t pcfs_open( void * file_p, const char * fn, lv_fs_mode_t mode)
{
    errno = 0;
    const char * flags = "";
    if(mode == LV_FS_MODE_WR) flags = "wb";
    else if(mode == LV_FS_MODE_RD) flags = "rb";
    else if(mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) flags = "a+";

    char buf[256];
    sprintf(buf, "/%s", fn);
    pc_file_t f = fopen(buf, flags);

    if(f == NULL)
      return LV_FS_RES_UNKNOWN;
    else {
      fseek(f, 0, SEEK_SET);
      pc_file_t * fp = (pc_file_t *)file_p;
      *fp = f;
    }

    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_close( void * file_p)
{
    pc_file_t * fp = (pc_file_t *)file_p;
    fclose(*fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_read( void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    pc_file_t * fp =  (pc_file_t *)file_p;
    *br = fread(buf, 1, btr, *fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_seek( void * file_p, uint32_t pos)
{
    pc_file_t * fp = (pc_file_t *)file_p;
    fseek(*fp, pos, SEEK_SET);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_tell( void * file_p, uint32_t * pos_p)
{
    pc_file_t * fp =  (pc_file_t *)file_p;
    *pos_p = ftell(*fp);
    return LV_FS_RES_OK;
}




static lv_res_t btnm_action(lv_obj_t *btnm, const char *txt){
	for (int i = 0; i < sizeof(auton_strings) / sizeof(auton_strings[0]); i++){
		if (strcmp(auton_strings[i], txt) == 0){
			autonNumber = i + 1;
			break;
		}
		lv_btnm_set_toggle(btnm, true, autonNumber);
	}

	return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}


void drawImage() {
	lv_fs_drv_t pcfs_drv;                         /*A driver descriptor*/
  memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));    /*Initialization*/

  pcfs_drv.file_size = sizeof(pc_file_t);       /*Set up fields...*/
  pcfs_drv.letter = 'S';
  pcfs_drv.open = pcfs_open;
  pcfs_drv.close = pcfs_close;
  pcfs_drv.read = pcfs_read;
  pcfs_drv.seek = pcfs_seek;
  pcfs_drv.tell = pcfs_tell;
  lv_fs_add_drv(&pcfs_drv);

	lv_obj_t * aditya_img = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(aditya_img, "S:/usd/aditya_picture.bin");
	lv_obj_set_pos(aditya_img, 0, 0);




}


void autonomousChooserInit(){
	lv_theme_alien_init(40, NULL);

	lv_obj_t *title = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(title, "6121B Auto Sauce");
	lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

	lv_obj_t *btnm = lv_btnm_create(lv_scr_act(), NULL);
	lv_btnm_set_map(btnm, btnm_map);
	lv_btnm_set_action(btnm, btnm_action);
	lv_obj_set_size(btnm, LV_HOR_RES - 20, LV_VER_RES / 2);
	lv_obj_align(btnm, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
}

void autonomousChooserExecuteAuto(){

	switch (autonNumber){
		case 1:
			rightBoth();
			break;
		case 2:
			leftBoth();
			break;
		case 3:
			rightNeutrals();
			break;
		case 4:
			leftNeutrals();
			break;
		case 5:
			soloAWP();
			break;
		case 6:
			rightRush();
			break;
		case 7:
			leftRush();
			break;
		case 8:
			rightMidRush();
			break;
		case 9:
			setDriverIntake(true);
			break;
		case 10:
			progSkills();
		default:
			break;
	}
}
