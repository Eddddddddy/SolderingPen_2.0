// 菜单项
const uint8_t language_types = 4;
//                                            "温控类型",   "溫控類型",   "Control Type",

const char *SetupItems[][language_types] = { "菜单设置", "菜單設置", "Setup Menu", "Configuración",
                                             "烙铁头设置", "烙鐵頭設置", "Tip Settings", "Punta",
                                             "温度设置", "溫度設置", "Temp Settings", "Temperatura",
                                             "时间设置", "時間設置", "Timer Settings", "Temporizadores",
                                             "主屏幕", "主屏幕", "Main Screen", "Pant. Principal",
                                             "信息", "信息", "Information", "Información",
                                             "电压设置", "電壓設置", "Voltage", "Voltaje",
                                             "QC", "QC", "QC", "QC",
                                             "蜂鸣器", "蜂鳴器", "Buzzer", "Zumbador",
                                             "恢复默认设置", "恢復默認設置", "Restore Config", "Restaurar Config",
                                             "更新版本", "更新版本", "Update Firmware", "Act. de Firmware",
                                             "Languages", "Languages", "Languages", "Idiomas",
                                             "切换左右手", "切换左右手", "L/R Hand", "Mano Dominante",
                                             "返回", "返回", "Return", "Volver" };
const char *LanguagesItems[][language_types] = { "Languages", "Languages", "Languages", "Idiomas",
                                                 "简体中文", "简体中文", "zh-CN", "es-CL",
                                                 "繁体中文", "繁体中文", "zh-TW", "es-CL",
                                                 "English", "English", "English", "Español" };
const char *TipItems[][language_types] = { "烙铁头:", "烙鐵頭:", "Tip:", "Punta:",
                                           "更换烙铁头", "更換烙鐵頭", "Change Tip", "Cambiar Punta",
                                           "校准烙铁头", "校準烙鐵頭", "Calibrate Tip", "Calibrar Punta",
                                           "重命名烙铁头", "重命名烙鐵頭", "Rename Tip", "Renombrar Punta",
                                           "删除烙铁头", "刪除烙鐵頭", "Delete Tip", "Eliminar Punta",
                                           "新建烙铁头", "新建烙鐵頭", "New Tip", "Nueva Punta",
                                           "返回", "返回", "Return", "Volver" };
const char *TempItems[][language_types] = { "温度设置", "溫度設置", "Temp Settings", "Temperatura",
                                            "默认温度", "默認溫度", "Default Temp", "Normal",
                                            "休眠温度", "休眠溫度", "Sleep Temp", "Modo Sueño",
                                            "提高温度", "提高溫度", "Boost Temp", "Aumentada",
                                            "返回", "返回", "Return", "Volver" };
const char *TimerItems[][language_types] = { "时间设置", "時間設置", "Timer Settings", "Temporizadores",
                                             "休眠时间", "休眠時間", "Sleep Timer", "Temp. Modo Sueño",
                                             "关闭时间", "關閉時間", "Shutdown Timer", "Temp. Apagado",
                                             "提温时间", "提溫時間", "Boost Timer", "Temp. Aumento",
                                             "唤醒阈值", "唤醒閾值", "Wake Threshold", "Umbral Encendido",
                                             "返回", "返回", "Return", "Volver" };
const char *ControlTypeItems[][language_types] = { "温控类型", "溫控類型", "Control Type", "Tipo de Control",
                                                   "非PID", "非PID", "Non-PID", "No PID",
                                                   "PID", "PID", "PID", "PID" };
const char *MainScreenItems[][language_types] = {
  "主屏幕", "主屏幕", "Main Screen", "Pant. Principal",
  "大数字", "大數字", "Big Number", "Números Grandes",
  "更多信息", "更多信息", "More Info", "Más Información",
  "高密度", "高密度", "High Density", "Alta Densidad",
};
const char *StoreItems[][language_types] = { "存储设置?", "存儲設置?", "Save?", "¿Guardar?",
                                             "否", "否", "No", "No",
                                             "是", "是", "Yes", "Sí" };
const char *DefaultItems[][language_types] = { "恢复设置?", "恢復設置?", "Restore?", "¿Restaurar?",
                                               "否", "否", "No", "No",
                                               "是", "是", "Yes", "Sí" };
const char *SureItems[][language_types] = { "确定?", "確定?", "Sure?", "¿Confirmar?",
                                            "否", "否", "No", "No",
                                            "是", "是", "Yes", "Sí" };
const char *VoltageItems[][language_types] = { "电压设置", "電壓設置", "Voltage Settings", "Voltaje",
                                               "9V", "9V", "9V", "9V",
                                               "12V", "12V", "12V", "12V",
                                               "15V", "15V", "15V", "15V",
                                               "20V(50%)", "20V(50%)", "20V(50%)", "20V(50%)",
                                               "20V(100%)", "20V(100%)", "20V(100%)", "20V(100%)"};
const char *QCItems[][language_types] = { "QC", "QC", "QC", "QC",
                                          "禁用", "禁用", "Disable", "Desactivar",
                                          "启用", "啟用", "Enable", "Activar" };
const char *BuzzerItems[][language_types] = { "蜂鸣器", "蜂鳴器", "Buzzer", "Zumbador",
                                              "禁用", "禁用", "Disable", "Desactivar",
                                              "启用", "啟用", "Enable", "Activar" };
const char *DefaultTempItems[][language_types] = { "默认温度", "默認溫度", "Default Temp", "Temp. Normal",
                                                   "deg C", "deg C", "deg C", "grados C" };
const char *SleepTempItems[][language_types] = { "休眠温度", "休眠溫度", "Sleep Temp", "Temp. Sueño",
                                                 "deg C", "deg C", "deg C", "grados C" };
const char *BoostTempItems[][language_types] = { "提高温度", "提高溫度", "Boost Temp", "Temp. Aumento",
                                                 "deg C", "deg C", "deg C", "grados C" };
const char *SleepTimerItems[][language_types] = { "休眠时间", "休眠時間", "Sleep Timer", "Tempor. Sueño",
                                                  "秒", "秒", "sec", "seg" };
const char *WAKEUPthresholdItems[][language_types] = { "唤醒阈值", "唤醒閾值", "Wake Threshold", "Umbral Despertar",
                                                       "mg", "mg", "mg", "mg" };
const char *OffTimerItems[][language_types] = { "关闭时间", "關閉時間", "Shutdown Timer", "Tempor. Apagado",
                                                "分钟", "分鐘", "min", "min" };
const char *BoostTimerItems[][language_types] = { "提温时间", "提溫時間", "Boost Timer", "Tempor. Aumento",
                                                  "秒", "秒", "sec", "seg" };
const char *DeleteMessage[][language_types] = { "警告", "警告", "Warning", "Advertencia",
                                                "你不能", "你不能", "You can't", "¡No puedes",
                                                "删除你的", "刪除你的", "delete your", "eliminar tu",
                                                "最后的烙铁头!", "最後的烙鐵頭!", "last tip!", "última punta!" };
const char *MaxTipMessage[][language_types] = { "警告", "警告", "Warning", "Advertencia",
                                                "你已达", "你已達", "You reached", "¡Has alcanzado",
                                                "最大数量", "最大數量", "maximum number", "el máximo",
                                                "的烙铁头!", "的烙鐵頭!", "of tips!", "de puntas!" };

const char *txt_set_temp[] = { "设温", "設溫", "SET", "TEMP" };
const char *txt_error[] = { "错误", "錯誤", "ERROR", "ERROR" };
const char *txt_off[] = { "关闭", "關閉", "OFF", "APAG" };
const char *txt_sleep[] = { "休眠", "休眠", "SLEEP", "SUEÑO" };
const char *txt_boost[] = { "提温", "提溫", "BOOST", "AUMEN" };
const char *txt_worky[] = { "工作", "工作", "WORK", "FUNC" };
const char *txt_on[] = { "加热", "加熱", "HEAT", "CALOR" };
const char *txt_hold[] = { "保持", "保持", "HOLD", "SUSP" };

const char *txt_Deactivated[] = { "禁用", "禁用", "Deactivated", "Desactivado" };

const char *txt_temp[] = { "温度:", "溫度:", "TEMP:", "TEMP:" };
const char *txt_voltage[] = { "电压:", "電壓:", "VOLT:", "VOLT:" };
const char *txt_Version[] = { "版本:", "版本:", "VER:", "VER:" };

const char *txt_select_tip[] = { "选择烙铁头", "選擇烙鐵頭", "Select Tip", "Elegir Punta" };

const char *txt_calibrate[] = { "校准", "校準", "Calibrate", "Calibrar" };
const char *txt_step[] = { "步进", "步進", "Step", "Paso" };
const char *txt_set_measured[] = { "设为测量", "設為測量", "Set Measure", "Medir" };
const char *txt_s_temp[] = { "的温度:", "的溫度:", "Temp:", "Temp:" };
const char *txt_temp_2[] = { "温度：  ", "溫度：  ", "Temp:  ", "Temp:  " };
const char *txt_wait_pls[] = { "请稍等...", "請稍等...", "Please wait ...", "Espera..." };

const char *txt_enter_tip_name[] = { "输入烙铁头名称", "輸入烙鐵頭名稱", "Enter Tip Name", "Ingresar Nombre" };
