#include <Keyboard.h>

// asetniop LUT
char asetniop_lut[256] = {
  /*ASET NIOP*/
  /*____ ____*/ 0,
  /*____ ___P*/ 'p',
  /*____ __O_*/ 'o',
  /*____ __OP*/ ';',	/* ; : */
  /*____ _I__*/ 'i',
  /*____ _I_P*/ '!',	/* ! @ */
  /*____ _IO_*/ 'l',
  /*____ _IOP*/ '1',	/* 1 ~ */
  /*____ N___*/ 'n',
  /*____ N__P*/ 'm',
  /*____ N_O_*/ 'u',
  /*____ N_OP*/ 0,
  /*____ NI__*/ 'h',
  /*____ NI_P*/ 0,
  /*____ NIO_*/ 0,
  /*____ NIOP*/ KEY_RETURN,
  /*___T ____*/ 't',
  /*___T ___P*/ KEY_BACKSPACE,
  /*___T __O_*/ 'g',
  /*___T __OP*/ 0,
  /*___T _I__*/ 'v',
  /*___T _I_P*/ 0,
  /*___T _IO_*/ 0,
  /*___T _IOP*/ '5',	/* 5 % */
  /*___T N___*/ 'b',
  /*___T N__P*/ 0,
  /*___T N_O_*/ 0,
  /*___T N_OP*/ 0,
  /*___T NI__*/ 0,
  /*___T NI_P*/ 0,
  /*___T NIO_*/ 0,
  /*___T NIOP*/ 0,
  /*__E_ ____*/ 'e',
  /*__E_ ___P*/ '=',	/* = + */
  /*__E_ __O_*/ '-',	/* - _ */
  /*__E_ __OP*/ 0,
  /*__E_ _I__*/ ',',	/* , < */
  /*__E_ _I_P*/ 0,
  /*__E_ _IO_*/ 0,
  /*__E_ _IOP*/ '4',	/* 4 $ */
  /*__E_ N___*/ 'y',
  /*__E_ N__P*/ 0,
  /*__E_ N_O_*/ 0,
  /*__E_ N_OP*/ 0,
  /*__E_ NI__*/ 0,
  /*__E_ NI_P*/ 0,
  /*__E_ NIO_*/ 0,
  /*__E_ NIOP*/ 0,
  /*__ET ____*/ 'r',
  /*__ET ___P*/ 0,
  /*__ET __O_*/ 0,
  /*__ET __OP*/ 0,
  /*__ET _I__*/ 0,
  /*__ET _I_P*/ 0,
  /*__ET _IO_*/ 0,
  /*__ET _IOP*/ 0,
  /*__ET N___*/ 0,
  /*__ET N__P*/ 0,
  /*__ET N_O_*/ 0,
  /*__ET N_OP*/ 0,
  /*__ET NI__*/ 0,
  /*__ET NI_P*/ 0,
  /*__ET NIO_*/ 0,
  /*__ET NIOP*/ 0,
  /*_S__ ____*/ 's',
  /*_S__ ___P*/ ')',	/* ) ] */
  /*_S__ __O_*/ '.',	/* . > */
  /*_S__ __OP*/ 0,
  /*_S__ _I__*/ 'k',
  /*_S__ _I_P*/ 0,
  /*_S__ _IO_*/ 0,
  /*_S__ _IOP*/ '3',	/* 3 # */
  /*_S__ N___*/ 'j',
  /*_S__ N__P*/ 0,
  /*_S__ N_O_*/ 0,
  /*_S__ N_OP*/ 0,
  /*_S__ NI__*/ 0,
  /*_S__ NI_P*/ 0,
  /*_S__ NIO_*/ 0,
  /*_S__ NIOP*/ 0,
  /*_S_T ____*/ 'c',
  /*_S_T ___P*/ 0,
  /*_S_T __O_*/ 0,
  /*_S_T __OP*/ 0,
  /*_S_T _I__*/ 0,
  /*_S_T _I_P*/ 0,
  /*_S_T _IO_*/ 0,
  /*_S_T _IOP*/ 0,
  /*_S_T N___*/ 0,
  /*_S_T N__P*/ 0,
  /*_S_T N_O_*/ 0,
  /*_S_T N_OP*/ 0,
  /*_S_T NI__*/ 0,
  /*_S_T NI_P*/ 0,
  /*_S_T NIO_*/ 0,
  /*_S_T NIOP*/ 0,
  /*_SE_ ____*/ 'd',
  /*_SE_ ___P*/ 0,
  /*_SE_ __O_*/ 0,
  /*_SE_ __OP*/ 0,
  /*_SE_ _I__*/ 0,
  /*_SE_ _I_P*/ 0,
  /*_SE_ _IO_*/ ' ',	/*SPACE*/
  /*_SE_ _IOP*/ 0,
  /*_SE_ N___*/ 0,
  /*_SE_ N__P*/ 0,
  /*_SE_ N_O_*/ 0,
  /*_SE_ N_OP*/ 0,
  /*_SE_ NI__*/ 0,
  /*_SE_ NI_P*/ 0,
  /*_SE_ NIO_*/ 0,
  /*_SE_ NIOP*/ 0,
  /*_SET ____*/ '6',	/* 6 ^ */
  /*_SET ___P*/ '7',	/* 7 & */
  /*_SET __O_*/ '8',	/* 8 * */
  /*_SET __OP*/ 0,
  /*_SET _I__*/ '9',	/* 9 { */
  /*_SET _I_P*/ 0,
  /*_SET _IO_*/ 0,
  /*_SET _IOP*/ 0,
  /*_SET N___*/ '0',	/* 0 } */
  /*_SET N__P*/ 0,
  /*_SET N_O_*/ 0,
  /*_SET N_OP*/ 0,
  /*_SET NI__*/ 0,
  /*_SET NI_P*/ 0,
  /*_SET NIO_*/ 0,
  /*_SET NIOP*/ 0,
  /*A___ ____*/ 'a',
  /*A___ ___P*/ '?',	/* ? / */
  /*A___ __O_*/ '(',	/* ( [ */
  /*A___ __OP*/ '\\',	/* \ | */
  /*A___ _I__*/ 'z',
  /*A___ _I_P*/ 0,
  /*A___ _IO_*/ 0,
  /*A___ _IOP*/ '2',	/* 2 @ */
  /*A___ N___*/ 'q',
  /*A___ N__P*/ 0,
  /*A___ N_O_*/ 0,
  /*A___ N_OP*/ 0,
  /*A___ NI__*/ 0,
  /*A___ NI_P*/ 0,
  /*A___ NIO_*/ 0,
  /*A___ NIOP*/ 0,
  /*A__T ____*/ 'f',
  /*A__T ___P*/ 0,
  /*A__T __O_*/ 0,
  /*A__T __OP*/ 0,
  /*A__T _I__*/ 0,
  /*A__T _I_P*/ 0,
  /*A__T _IO_*/ 0,
  /*A__T _IOP*/ 0,
  /*A__T N___*/ 0,
  /*A__T N__P*/ 0,
  /*A__T N_O_*/ 0,
  /*A__T N_OP*/ 0,
  /*A__T NI__*/ 0,
  /*A__T NI_P*/ 0,
  /*A__T NIO_*/ 0,
  /*A__T NIOP*/ 0,
  /*A_E_ ____*/ 'x',
  /*A_E_ ___P*/ 0,
  /*A_E_ __O_*/ 0,
  /*A_E_ __OP*/ 0,
  /*A_E_ _I__*/ 0,
  /*A_E_ _I_P*/ 0,
  /*A_E_ _IO_*/ 0,
  /*A_E_ _IOP*/ 0,
  /*A_E_ N___*/ 0,
  /*A_E_ N__P*/ 0,
  /*A_E_ N_O_*/ 0,
  /*A_E_ N_OP*/ 0,
  /*A_E_ NI__*/ 0,
  /*A_E_ NI_P*/ 0,
  /*A_E_ NIO_*/ 0,
  /*A_E_ NIOP*/ 0,
  /*A_ET ____*/ 0,
  /*A_ET ___P*/ 0,
  /*A_ET __O_*/ 0,
  /*A_ET __OP*/ 0,
  /*A_ET _I__*/ 0,
  /*A_ET _I_P*/ 0,
  /*A_ET _IO_*/ 0,
  /*A_ET _IOP*/ 0,
  /*A_ET N___*/ 0,
  /*A_ET N__P*/ 0,
  /*A_ET N_O_*/ 0,
  /*A_ET N_OP*/ 0,
  /*A_ET NI__*/ 0,
  /*A_ET NI_P*/ 0,
  /*A_ET NIO_*/ 0,
  /*A_ET NIOP*/ 0,
  /*AS__ ____*/ 'w',
  /*AS__ ___P*/ 0,
  /*AS__ __O_*/ 0,
  /*AS__ __OP*/ KEY_DELETE,
  /*AS__ _I__*/ 0,
  /*AS__ _I_P*/ 0,
  /*AS__ _IO_*/ 0,
  /*AS__ _IOP*/ 0,
  /*AS__ N___*/ 0,
  /*AS__ N__P*/ 0,
  /*AS__ N_O_*/ 0,
  /*AS__ N_OP*/ 0,
  /*AS__ NI__*/ 0,
  /*AS__ NI_P*/ 0,
  /*AS__ NIO_*/ 0,
  /*AS__ NIOP*/ 0,
  /*AS_T ____*/ 0,
  /*AS_T ___P*/ 0,
  /*AS_T __O_*/ 0,
  /*AS_T __OP*/ 0,
  /*AS_T _I__*/ 0,
  /*AS_T _I_P*/ 0,
  /*AS_T _IO_*/ 0,
  /*AS_T _IOP*/ 0,
  /*AS_T N___*/ 0,
  /*AS_T N__P*/ 0,
  /*AS_T N_O_*/ 0,
  /*AS_T N_OP*/ 0,
  /*AS_T NI__*/ 0,
  /*AS_T NI_P*/ 0,
  /*AS_T NIO_*/ 0,
  /*AS_T NIOP*/ 0,
  /*ASE_ ____*/ 0,
  /*ASE_ ___P*/ 0,
  /*ASE_ __O_*/ 0,
  /*ASE_ __OP*/ 0,
  /*ASE_ _I__*/ 0,
  /*ASE_ _I_P*/ 0,
  /*ASE_ _IO_*/ 0,
  /*ASE_ _IOP*/ 0,
  /*ASE_ N___*/ 0,
  /*ASE_ N__P*/ 0,
  /*ASE_ N_O_*/ 0,
  /*ASE_ N_OP*/ 0,
  /*ASE_ NI__*/ 0,
  /*ASE_ NI_P*/ 0,
  /*ASE_ NIO_*/ 0,
  /*ASE_ NIOP*/ 0,
  /*ASET ____*/ KEY_TAB,
  /*ASET ___P*/ KEY_END,
  /*ASET __O_*/ KEY_HOME,
  /*ASET __OP*/ 0,
  /*ASET _I__*/ 0,
  /*ASET _I_P*/ KEY_PAGE_DOWN,
  /*ASET _IO_*/ 0,
  /*ASET _IOP*/ 0,
  /*ASET N___*/ 0,
  /*ASET N__P*/ KEY_PAGE_UP,
  /*ASET N_O_*/ 0,
  /*ASET N_OP*/ 0,
  /*ASET NI__*/ 0,
  /*ASET NI_P*/ 0,
  /*ASET NIO_*/ 0,
  /*ASET NIOP*/ 0,
};
