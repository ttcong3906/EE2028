/* Real-time runner: firmware UART on a pty, sim inputs on a control FIFO.
 *   simrt <uart-pty> <control-fifo> <flash-file>
 * Control lines: press | dbl | flip | unflip | ir | tilt ax ay az (mg) | fw <idx> <ms> | quit
 *
 * Build (after run_tests.sh created build/):  gcc -std=gnu11 -Istub -Ibuild -o build/simrt simrt.c -lm
 * Wire it to the dashboard on Linux:
 *   socat pty,raw,echo=0,link=/tmp/drone pty,raw,echo=0,link=/tmp/drone_fw &
 *   mkfifo /tmp/drone.in; (sleep 1e9 > /tmp/drone.in) &
 *   build/simrt /tmp/drone_fw /tmp/drone.in /tmp/drone.flash &
 *   python3 ../dashboard.py --drone /tmp/drone
 *   echo dbl > /tmp/drone.in      # double press -> BATTLE;  echo flip > /tmp/drone.in ... */
#define SIM_RT
#include "sim.c"
#include <sys/stat.h>
#include <errno.h>

static void rt_flash_save(void)
{
    if (!rt_flash_file) return;
    FILE *f = fopen(rt_flash_file, "wb");
    if (f) { fwrite(sim_flash_page, 1, sizeof sim_flash_page, f); fclose(f); }
}

static uint32_t press_until = 0, press2_at = 0, fw_until = 0;

static void control(char *l)
{
    int a, b, c;
    if (!strncmp(l, "press", 5))      { btn_edge(0); press_until = tick + 80; }
    else if (!strncmp(l, "dbl", 3))   { btn_edge(0); press_until = tick + 60; press2_at = tick + 200; }
    else if (!strncmp(l, "unflip", 6)) s_acc[2] = 1000;
    else if (!strncmp(l, "flip", 4))  s_acc[2] = -1000;
    else if (!strncmp(l, "ir", 2))    { ir_pin = 0; }
    else if (sscanf(l, "tilt %d %d %d", &a, &b, &c) == 3) { s_acc[0] = (int16_t)a; s_acc[1] = (int16_t)b; s_acc[2] = (int16_t)c; }
    else if (sscanf(l, "fw %d %d", &a, &b) == 2) { sim_fw = (uint8_t)(1u << a); fw_until = tick + (uint32_t)b; }
    else if (!strncmp(l, "quit", 4))  exit(0);
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: simrt <pty> <fifo> <flash>\n"); return 2; }
    rt_flash_file = argv[3];
    memset(sim_flash_page, 0xFF, sizeof sim_flash_page);
    FILE *ff = fopen(rt_flash_file, "rb");
    if (ff) { size_t n = fread(sim_flash_page, 1, sizeof sim_flash_page, ff); (void)n; fclose(ff); }
    rt_fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (rt_fd < 0) { perror("pty"); return 1; }
    struct termios tio; if (tcgetattr(rt_fd, &tio) == 0) { cfmakeraw(&tio); tcsetattr(rt_fd, TCSANOW, &tio); }
    int cf = open(argv[2], O_RDONLY | O_NONBLOCK);
    if (cf < 0) { perror("fifo"); return 1; }
    char cbuf[256]; size_t cn = 0;

    int j = setjmp(jmp_end);
    if (j == 2) { rt_flash_save(); close(rt_fd); close(cf); execv(argv[0], argv); perror("execv"); return 1; }
    if (j == 1) { for (;;) pause(); }

    HAL_GetTick();
    cfg_load();
    if (cfg.role == ROLE_CONTROLLER) { ctl_mode = 1; ctl_init(); } else app_init();
    for (;;) {
        HAL_GetTick();
        uint8_t ch;
        while (read(rt_fd, &ch, 1) == 1) { *rx_ptr = ch; HAL_UART_RxCpltCallback(&huart1); }
        ssize_t r = read(cf, cbuf + cn, sizeof cbuf - 1 - cn);
        if (r > 0) {
            cn += (size_t)r; cbuf[cn] = 0;
            char *nl;
            while ((nl = strchr(cbuf, '\n'))) { *nl = 0; control(cbuf); memmove(cbuf, nl + 1, strlen(nl + 1) + 1); cn = strlen(cbuf); }
        }
        if (press_until && tick >= press_until) { btn_edge(1); press_until = 0; }
        if (press2_at && tick >= press2_at) { btn_edge(0); press_until = tick + 60; press2_at = 0; }
        if (fw_until && tick >= fw_until) { sim_fw = 0; fw_until = 0; }
        if (ir_pin == 0) { static uint32_t t_ir0 = 0; if (!t_ir0) t_ir0 = tick; if (tick - t_ir0 > 150) { ir_pin = 1; t_ir0 = 0; } }
        if (ctl_mode) ctl_loop(); else app_loop();
        service_isrs();
        usleep(300);
    }
}
