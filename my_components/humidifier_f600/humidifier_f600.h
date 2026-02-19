#pragma once

#ifndef HUMIDIFIER_F600_H
#define HUMIDIFIER_F600_H


#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/htu21d/htu21d.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/version.h"
#include "esphome/core/util.h"

#if __has_include("esphome/core/macros.h")
   #include "esphome/core/macros.h" // VERSION_CODE
#else
   #define VERSION_CODE(major, minor, patch) ((major) << 16 | (minor) << 8 | (patch))
#endif

#ifdef ESP32
    #include "esphome/core/preferences.h"
#else
    #warning "Saving presets does not work with ESP8266" 
#endif    

#include <queue>

namespace esphome {
namespace humidifier_f600 {

using namespace esphome;
using namespace esphome::switch_;
using namespace esphome::binary_sensor;
using namespace esphome::text_sensor;
using namespace esphome::sensor;
using namespace esphome::select;
using namespace esphome::number;
using namespace esphome::htu21d;

// операции с сенсором
enum sens_op:uint32_t { STATUS=0, // флаг проверки текущего статуса кнопки
                        CLICK=20, // клик по сенсору 20
                        NEW_CLICK=100, // клик для новой генераци устройств
                        PUSH=3500, // долгое нажатие для выключения
};
// текущий статус работы
enum device_op:uint8_t  { doIDLE,  // простой
                          doLOW,   // cлабое распыление
                          doMID,   // среднее распыление
                          doHIGH,  // сильное распыление
                          doERROR, // ошибка, возможно нет воды
                          doUNDEF,
                          doMANUAL_OFF, // пользователь перехватил управление
                          doMANUAL_LOW, // пользователь перехватил управление
                          doMANUAL_MID, // пользователь перехватил управление
                          doMANUAL_HIGH, // пользователь перехватил управление
};
// установка пользователя
enum device_set:uint8_t { dsOFF,
                          dsLOW,
                          dsMID,
                          dsHIGH,
                          dsAUTO,
                          dsUNDEF
};
// структура для сохранения данных
struct save_struct { 
    device_set preset=dsOFF; // установка пользователя
    float humidity=45; // уставка влажности
};

class HumiF600;

class HumiF600PresetSelect : public esphome::select::Select, public esphome::Parented<HumiF600> {
    protected:
     #if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
	    void control(size_t index) override{
            this->state_callback_.call(index);
        }
     #else
	    void control(const std::string &value) override{
            this->state_callback_.call(value, index_of(value).value());
        }
     #endif
    friend class HumiF600;   
};

class HumiF600TargetNumber : public esphome::number::Number, public esphome::Parented<HumiF600> {
    protected:
	    void control(float value) override{
            this->state_callback_.call(value);
        }
    friend class HumiF600;   
};

class HumiF600 : public Sensor, public PollingComponent  {
  private:
    const char *const TAG = "Deerma_F600";
    const char *const _Idle = "Idle";
    const char *const _Low = "Low";
    const char *const _Middle = "Medium";
    const char *const _High = "High";
    const char *const _Unexpected = "Unexpected";
    const char *const _Off = "Off";
    const char *const _Auto = "Auto";
    const char *const _Error = "Error";
    const char *const _ManualOff = "Manual Off";
    const char *const _ManualLow = "Manual Low";
    const char *const _ManualMid = "Manual Mid";
    const char *const _ManualHigh = "Manual High";
    // для автоматического регулирования влажности
    const float gisteresis=2.0; //гистерезис регулирования
    // для регулировани режимов в пресете AUTO
    const float set_high=12.0; //порог работы на максимуме, переключится на средний 
    const float set_mid=6.0; //порог работы на средних настройках , переключится 
    const uint32_t changeDelay=30000; // после переключения в режиме авто 30 сек ничего не меняем
  
    save_struct store_data; // массив сохрнения данных
    #ifdef ESP32    
       #if ESPHOME_VERSION_CODE > VERSION_CODE(2026, 1, 5)
           ESPPreferenceObject storage = this->make_entity_preference<save_struct>();
       #else
           ESPPreferenceObject storage = global_preferences->make_preference<save_struct>(this->get_object_id_hash(), true);
       #endif
        uint8_t load_presets_result = 0xFF;
    #endif
    
    uint8_t now_temperature=0; // показания локальной температуры
    device_op now_operate=doUNDEF; //режим работы
    device_op save_operate=doMID; //сохраненный режим , для запуска после работы
    bool now_water=true;  //наличие воды    
    char scr[3]={0};      //текущая надпись на дисплее
    int16_t LCDetect=0;  // для детекции типа LCD (китайцы изменили схему подключения дисплея или тип дсплея, а значит и управление им)
    uint8_t LCDerror=0; // счетчк ошибок чтения дсплея
    uint32_t chScrTimer=0; //таймер последнего изменения режима экрана 
    uint32_t sens_timer=0; //таймер нажатия на сенсор
    uint32_t gap_timer=1500; //тайминг паузы между нажатям
    uint32_t sens_delay=0; //установка задержка нажатия на сенсор
    device_set now_set=store_data.preset; //текущая установка пользователя
    device_set target_set=dsUNDEF; // желаемый режим работы
    bool need_new_set = false; // необходимость установки запрошенного по сети режима
    float target_hummidity_=store_data.humidity; // целевая влажность
    float hummidity_=0; // текущая влажность
    bool manual=false; // флаг перехвата в ручное управление, поднимается при работе кнопками на устройстве 
    
    #ifdef ESP32
        // сохранение данные в епроме
        void store(){ 
            if(store_data.preset!=now_set || store_data.humidity!=target_hummidity_){ // проверка реального изменения данных
                store_data.preset=now_set;
                store_data.humidity=target_hummidity_;
                if(storage.save(&store_data)){
                    if(global_preferences->sync()){ // сохраняем во флеш
                        ESP_LOGD(TAG,"Sync NVRAM OK (load result: %02d)",load_presets_result);
                    } else {
                        ESP_LOGE(TAG,"Save NVRAM ERROR (load result: %02d)",load_presets_result);
                    }
                }
            }
        }
        
        // восстановление данных из епрома
        void restore(){
            load_presets_result = storage.load(&store_data); // читаем из флеша
            ESP_LOGD(TAG,"Preset base read from NVRAM, result %02d",load_presets_result);
            now_set=store_data.preset;
            target_hummidity_=store_data.humidity;
        }
    #else
        void store(){;}
        void restore(){;}
    #endif
    // распознавание считанных символов из кода 7 сегментного индикатора в символы
    
    uint8_t getNum(uint8_t d){
        if(LCDetect>=10){ // детектирована старая схема подключения
           if(d<0x20){
              if(d==0x00) return '8'; 
              else if(d==0x03) return '6'; 
              else if(d==0x07) return 'E'; 
              else if(d==0x10) return '9'; 
              else if(d==0x13) return '5'; 
           } else if(d<0x90){
              if(d==0x24) return '2'; 
              else if(d==0x30) return '3'; 
              else if(d==0x37) return '#'; 
              else if(d==0x40) return '0'; 
              else if(d==0x78) return '7'; 
           } else {
              if(d==0x98) return '4'; 
              else if(d==0xB7) return '='; 
              else if(d==0xF7) return '_'; 
              else if(d==0xF8) return '1'; 
              else if(d==0xFF) return ' '; 
           }
        } else if(LCDetect<=-10){ // детектирована новая схема подключения
           if(d<0x80){
              if(d==0x00) return '8'; 
              else if(d==0x04) return '0'; 
              else if(d==0x10) return '6'; 
              else if(d==0x23) return '2'; 
              else if(d==0x30) return 'E'; 
           } else if(d<0xbf){
              if(d==0x80) return '9'; 
              else if(d==0x83) return '3'; 
              else if(d==0x90) return '5'; 
              else if(d==0xb3) return '#'; 
              else if(d==0xbb) return '='; 
           } else {   
              if(d==0xbf) return '_'; 
              else if(d==0xc7) return '7'; 
              else if(d==0xc8) return '4'; 
              else if(d==0xcf) return '1'; 
              else if(d==0xFF) return ' '; 
           } 
        } else { // фаза детекции схемы подключения дисплея
           if (d<0x11) {
              if(d==0x00) return '8';
              else if(d==0x03){LCDetect++;  return '6';} 
              else if(d==0x04){LCDetect--;  return '0';} 
              else if(d==0x07){LCDetect++;  return 'E';} 
              else if(d==0x10){
                 if     (LCDetect<0) return '6'; 
                 else if(LCDetect>0) return '9'; 
              }
           } else if(d<0x40) {
              if(d==0x13){LCDetect++;  return '5';} 
              else if(d==0x23){LCDetect--;  return '2';} 
              else if(d==0x24){LCDetect++;  return '2';} 
              else if(d==0x30){
                 if     (LCDetect>0) return '3'; 
                 else if(LCDetect<0) return 'E'; 
              }
              else if(d==0x37){LCDetect++;  return '#';} 
           } else if(d<0x91) {
              if(d==0x40){LCDetect++;  return '0';} 
              else if(d==0x78){LCDetect++;  return '7';} 
              else if(d==0x80){LCDetect--;  return '9';} 
              else if(d==0x83){LCDetect--;  return '3';} 
              else if(d==0x90){LCDetect--;  return '5';} 
           } else if(d<0xC0) {
              if(d==0x98){LCDetect++;  return '4';} 
              else if(d==0xb3){LCDetect--;  return '#';} 
              else if(d==0xB7){LCDetect++;  return '=';} 
              else if(d==0xbb){LCDetect--;  return '=';} 
              else if(d==0xbf){LCDetect--;  return '_';} 
           } else if(d<0xd0) {
              if(d==0xc7){LCDetect--;  return '7';} 
              else if(d==0xc8){LCDetect--;  return '4';} 
              else if(d==0xcf){LCDetect--;  return '1';} 
           } else {
              if(d==0xF7){LCDetect++;  return '_';} 
              else if(d==0xF8){LCDetect++;  return '1';} 
              else if(d==0xFF) return ' '; 
           }
        }
        return 0;
    }
    
    // считывание дисплея
    bool getStrFromLed(){
        if(this->disp_sync_pin->digital_read()){ // запрос в неудачное время, синхра в высоком состоянии
            return false;
        }
        static bool checkTiming=true; // флаг засечки тайминга синхры
        static uint32_t timeCalibrate=1950; // буфер тайминга синхры 
        uint32_t breakTimer=millis();
        while(!(this->disp_sync_pin->digital_read())){ //ждем когда синхра перекинется из 0 в 1
            if(millis()-breakTimer>50){ //предохранитель от зацикливания
                ESP_LOGE(TAG,"Display read start sync (0->1) ERROR !");
                return false;
            }
        }
        if(checkTiming){ // проход засечки тайминга
            timeCalibrate=micros();
            checkTiming=false;
            breakTimer=millis();
            while(this->disp_sync_pin->digital_read()){ // ждем когда синхра перекинется из 1 в 0
                if(millis()-breakTimer>5){ //предохранитель от зацикливания
                    ESP_LOGE(TAG,"Display read start sync (1->0) ERROR !");
                    return false;
                }
            }
            timeCalibrate=micros()-timeCalibrate-55; // получили тайминг бита
            return false;
        }
        // фаза чтение индикатора
        checkTiming=true;
        uint8_t mask=1;
        uint8_t d[3]={0};
        uint8_t t[2];
        uint32_t timer; // таймер синхронизации
        for(uint8_t i=0; i<8; i++){
            timer=micros();
            while(micros()-timer<40);
            if(this->disp_read0_pin->digital_read()){ // читаем данные первого канала
                d[0]|=mask;  
            }
            if(this->disp_read1_pin->digital_read()){ // данные второго канала
                d[1]|=mask;  
            }
            mask<<=1;
            while(micros()-timer<timeCalibrate); //ждем таймслота следующего бита
        }
        t[1]=getNum(d[1]); //во втором знакоместе совпадащх символов у нового и старого дисплея меньше   
        t[0]=getNum(d[0]);
        if(t[1]==0) t[1]=getNum(d[1]); // повторная попытка распознать первый символ
        if(t[0]==0 || t[1]==0){ // ошиблись
            LCDerror++; //увеличиваем счетчк ошибок
            if(LCDerror>8){
               ESP_LOGE(TAG,"Display read ERROR %x - %x ",d[0],d[1]);
               LCDerror--;
            } else {
               ESP_LOGV(TAG,"Display read ERROR %x - %x ",d[0],d[1]);
            }
            return false;
        }
        LCDerror=0; // сброс счетчиика ошбок
        if(scr[0]!=t[0] || scr[1]!=t[1]){ 
            scr[0]=t[0];
            scr[1]=t[1];
            ESP_LOGD(TAG,"Display: |%s|",scr);
            return true; // дисплей изменился
        }
        return false;  // дисплей не изменилось 
    }
    
    // эмуляция нажатия на сенсор , false- кнопка занята
    bool sensOn(sens_op delay){
        if(sens_delay){
            return false;
        } else if(delay==STATUS){
            return true; // проверка занятости кнопки
        }
        if(delay==PUSH){
           gap_timer = 6000; // пауза между нажатиями при выключении, поскольку выключение долго детектируется
        } else {
           gap_timer = 1500; // пауза между нажатиями
        }
        if(this->sens_pin2!=nullptr){ // есть второй сенсор, значит управляем правой кнопкой, включаем левой
           if(delay != PUSH && now_operate!=doIDLE){ // хотим выкл, при наличии второго сенсора это делается первым, но коротким кликом
              this->sens_pin2->digital_write(false); // тянем конденсатор на сенсоре к нулю
              this->sens_pin2_state=false;
              sens_timer = millis(); //таймер нажатия на сенсор
              sens_delay = NEW_CLICK; //установка задержки нажатия на сенсор
              ESP_LOGD(TAG,"%d Push sensor RIGHT, delay: %d",millis(),delay);
              return true;
           } else {
              delay=NEW_CLICK; //установка задержки нажатия на 1 сенсор
           }
        }        
        this->sens_pin->digital_write(false); // тянем конденсатор на сенсоре к нулю
        this->sens_pin_state=false;
        sens_timer = millis(); //таймер нажатия на сенсор
        sens_delay = delay; //установка задержки нажатия на сенсор
        ESP_LOGD(TAG,"%d Push sensor LEFT, delay: %d",millis(),delay);
        return true;
    }
 
    // текстовая расшифровка режима работы
    char* get_str_from_op(device_op op){
        static char ret[20]={0};
        if(op==doIDLE){
            strcpy(ret,_Idle);
        } else if(op==doLOW){
            strcpy(ret,_Low);
        } else if(op==doMID){
            strcpy(ret,_Middle);
        } else if(op==doHIGH){
            strcpy(ret,_High);
        } else if(op==doERROR){
            strcpy(ret,_Error);
        } else if(op==doMANUAL_OFF){
            strcpy(ret,_ManualOff);
        } else if(op==doMANUAL_LOW){
            strcpy(ret,_ManualLow);
        } else if(op==doMANUAL_MID){
            strcpy(ret,_ManualMid);
        } else if(op==doMANUAL_HIGH){
            strcpy(ret,_ManualHigh);
        } else {
            strcpy(ret,_Unexpected); 
        }
        return ret;
    }
  
    char* get_str_from_op_manual(device_op op){
        if(op==doIDLE){
            op=doMANUAL_OFF;
        } else if(op==doLOW){
            op=doMANUAL_LOW;
        } else if(op==doMID){
            op=doMANUAL_MID;
        } else if(op==doHIGH){
            op=doMANUAL_HIGH;
        }
        return get_str_from_op(op);
    }
  
    // определение режима работы установленного пользователем из стринга
    device_set get_set_from_str(const char* in){
        device_set set=dsUNDEF;
        if(strcmp(in,_Off)==0){
           set=dsOFF;
        } else if(strcmp(in,_Low)==0){
           set=dsLOW;
        } else if(strcmp(in,_Middle)==0){
           set=dsMID;
        } else if(strcmp(in,_High)==0){
           set=dsHIGH;
        } else if(strcmp(in,_Auto)==0){
           set=dsAUTO;
        }
        return set;
    }

    // определение stringa по номеру
    char* get_str_from_set(device_set set){
        static char ret[20]={0};
        if(set==dsOFF){
            strcpy(ret,_Off);
        } else if(set==dsLOW){
            strcpy(ret,_Low);
        } else if(set==dsMID){
            strcpy(ret,_Middle);
        } else if(set==dsHIGH){
            strcpy(ret,_High);
        } else if(set==dsAUTO){
            strcpy(ret,_Auto);
        } else {
            strcpy(ret,_Unexpected); 
        }
        return ret;
    }
        
    // обработка при изменении влажностей целевой и/или текущей
    void change_process(){
        ESP_LOGD(TAG,"Humiditys current: %3.1f, destination:%3.1f",hummidity_,target_hummidity_);
        if(manual==false){ // только не в режиме ручного управления
            if(hummidity_>(target_hummidity_+gisteresis)){ //влажность превысила установку
               if(now_operate!=doIDLE){ //если не в простое
                    target_set=dsOFF; // нужно отключить устройство
                    need_new_set=true;
                    ESP_LOGD(TAG,"Stop humiditing");
                }
            } else if(hummidity_<(target_hummidity_-gisteresis)){ //влажность меньше установленой
                if(now_set==dsAUTO){ // автоматический режим
                    float delta=target_hummidity_-hummidity_; // разница между влажностями
                    ESP_LOGD(TAG,"Mode AUTO, delta humidity:%3.1f",delta);
                    static float old_delta=delta;
                    static uint32_t timer=-changeDelay; // таймер гарантированной работы на этой настройке
                    if(abs(old_delta-delta)>gisteresis && millis()-timer>changeDelay){ // проверка гистерезиса, если переключились не менее 30 сек назад                   
                       if(delta>set_high && (now_operate==doIDLE || now_operate==doUNDEF || now_operate==doLOW || now_operate==doMID)){
                           target_set=dsHIGH; // включаем в максимальный режим
                           need_new_set=true;
                           ESP_LOGD(TAG,"Run HIGH humiditing");
                           timer=millis();
                       } else if(delta>set_mid && (now_operate==doIDLE || now_operate==doUNDEF || now_operate==doLOW || now_operate==doHIGH)){
                           target_set=dsMID; // включаем в средний режим
                           need_new_set=true;
                           ESP_LOGD(TAG,"Run MEDIUM humiditing");
                           timer=millis();
                       } else if(delta<set_mid && ( now_operate==doIDLE || now_operate==doUNDEF || now_operate==doMID || now_operate==doHIGH)){
                           target_set=dsLOW; // включаем в слабый режим
                           need_new_set=true;
                           ESP_LOGD(TAG,"Run LOW humiditing");
                           timer=millis();
                       }
                       old_delta=delta;
                    }
                } else if(/*(now_operate==doIDLE || now_operate==doUNDEF) && */ (now_set==dsLOW || now_set==dsMID || now_set==dsHIGH)){
                    need_new_set=true;
                    ESP_LOGD(TAG,"Run static humiditing");
                }
            }
        } else {
            ESP_LOGD(TAG,"In manual mode, the adjustment is disabled.");
        }
    }

    BinarySensor *water_ok{nullptr  };  // датчик наличия воды
    Sensor* ext_humi_sens{nullptr};     // внешний сенсор влажности
    Sensor* main_temp{nullptr};         // температура с внутреннего датчика увлажнителя
    GPIOPin* disp_sync_pin{nullptr};    // нога синхронизации дисплея
    GPIOPin* disp_read0_pin{nullptr};   // нога чтения данных первого знакоместа
    GPIOPin* disp_read1_pin{nullptr};   // нога чтения данных второго знакоместа
    GPIOPin* sens_pin{nullptr};         // нога управления сенсором
    bool sens_pin_state=true;
    GPIOPin* sens_pin2{nullptr};        // вторая нога управления сенсором
    bool sens_pin2_state=true;
    HumiF600TargetNumber *target_humidity{nullptr};   // целевая влажность
    TextSensor *op_mode{nullptr};       // текущий режим работы
  
  public:
    HumiF600PresetSelect *mode_select = nullptr;// режим работы
    // подключение ноги синхронизации дисплея
    void set_sync_pin(GPIOPin  *pin = nullptr){ this->disp_sync_pin=pin; pin->setup(); pin->pin_mode(gpio::FLAG_INPUT);} 
    // подключение ноги чтения первого знакоместа
    void set_read0_pin(GPIOPin  *pin = nullptr){ this->disp_read0_pin=pin; pin->setup(); pin->pin_mode(gpio::FLAG_INPUT);} 
    // подключение ноги чтения второго знакоместа
    void set_read1_pin(GPIOPin  *pin = nullptr){ this->disp_read1_pin=pin;  pin->setup(); pin->pin_mode(gpio::FLAG_INPUT);} 
    // подключение ноги управления сенсором
    void set_sens_pin(GPIOPin  *pin = nullptr){ this->sens_pin=pin; pin->setup(); pin->pin_mode(gpio::FLAG_OUTPUT); pin->digital_write(true);} 
    // подключение ноги управления вторым сенсором
    void set_sens2_pin(GPIOPin  *pin = nullptr){ 
       this->sens_pin2=pin; 
       pin->setup(); 
       pin->pin_mode(gpio::FLAG_OUTPUT); 
       pin->digital_write(true);
    } 
    // локальный сенсор температуры
    void set_main_temp(sensor::Sensor *main_sensor) { this->main_temp=main_sensor;} 
    // подключение внешнего сенсора влажности 
    void set_ext_humi_sensor(sensor::Sensor *ext_sensor) {
        this->ext_humi_sens = ext_sensor;
        this->ext_humi_sens->add_on_state_callback([this](float sensor_value) {
            // кoлбэк функция вызывается при публикации данных о влажности внешним сенсором
            if (!std::isnan(sensor_value)) {
                hummidity_=sensor_value; // текущая влажность
                change_process();
            }
        });
    }
    // подключение датчика текущего режима
    void set_text_operate(TextSensor* sens){op_mode=sens; op_mode->publish_state(get_str_from_op(doIDLE));}
    // подключение датчика наличия воды
    void set_water_sensor(BinarySensor* sens){
        this->water_ok=sens;
        this->water_ok->publish_state(this->now_water);
    }
    // режим установки пользователем
    void set_mode_select(HumiF600PresetSelect *sel) {
        this->mode_select = sel;
        restore();
        this->mode_select->publish_state(get_str_from_set(now_set));
      #if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
        this->mode_select->add_on_state_callback([this](size_t num) {
            std::string payload="";
            auto value = this->mode_select->at(num);
            if (value.has_value()) { 
               payload = value.value(); 
            } 
      #else
        this->mode_select->add_on_state_callback([this](std::string payload, size_t num) {
      #endif
            // колбэк при выборе режима 
            if(manual){
                manual=false; // отключаем ручное управление
                ESP_LOGD(TAG,"Manual mode Off");
            }
            device_set new_set=get_set_from_str(payload.c_str());
            now_set=new_set; // запоминаем режим для дальнейшей обработки, в режиме регулировки влажности
            if(now_set==dsAUTO){ // в автоматическом режиме начинаем с HIGH
                new_set=dsHIGH;   
            }
            target_set=new_set; // установим новый режим
            store();
            need_new_set = true; // необходимость установки запрошенного по сети режима
            ESP_LOGD(TAG,"User set mode: %s", payload.c_str());
            now_water=true; // для проверки наличия воды
            water_ok->publish_state(now_water);
        });
    }
    // подключение регулировки целевой влажности
    void set_humidity_dest(HumiF600TargetNumber *num){
        this->target_humidity=num;
        restore();
        this->target_humidity->state=this->target_hummidity_; //начальная инициализация
        this->target_humidity->publish_state(this->target_humidity->state);
        this->target_humidity->add_on_state_callback([this](float sensor_value) {
            // калбэк функция вызывается при публикации данных об установке влажности
            if (!std::isnan(sensor_value)) {
                if(manual){
                    manual=false; // отключаем ручное управление
                    ESP_LOGD(TAG,"Manual mode Off");
                }
                target_hummidity_= sensor_value; // целевая влажность
                store();
                change_process();
            }
        });
    }
    
    void dump_config() override {
      ESP_LOGCONFIG(TAG, "DEERMA F600:");
      LOG_PIN("Input display sync pin ", this->disp_sync_pin);
      LOG_PIN("Input display data0 pin ", this->disp_read0_pin);
      LOG_PIN("Input display data1 pin ", this->disp_read1_pin);
      LOG_PIN("Output sensor LEFT control pin ", this->sens_pin);
      if(this->sens_pin2!=nullptr){
         LOG_PIN("Output sensor RIGHT control pin ", this->sens_pin2);
      }
      LOG_SENSOR("", "Main Temperature ", this->main_temp);
      LOG_SENSOR("", "External Humidity ", this->ext_humi_sens);
      LOG_SELECT("", "Select mode ", this->mode_select);
      LOG_BINARY_SENSOR("", "Water sensor ", this->water_ok);   
      LOG_TEXT_SENSOR("", "Operating mode ", this->op_mode); 
      LOG_NUMBER("", "Humidity Target ", this->target_humidity);
      LOG_UPDATE_INTERVAL(this);
      ESP_LOGCONFIG(TAG, "Stored parameters:");
      ESP_LOGCONFIG(TAG, "  Target humidity %f",this->target_hummidity_);
      ESP_LOGCONFIG(TAG, "  Current mode %s",get_str_from_set(now_set));
    }
 
    float get_setup_priority() const override {return setup_priority::LATE;}  
     
    void setup() override {
        restore(); // восстановить настройки
        this->target_humidity->state=this->target_hummidity_; //начальная инициализация
        this->target_humidity->publish_state(this->target_humidity->state);
        this->mode_select->publish_state(get_str_from_set(now_set));
        this->change_process();
    }
    
    void loop() override {
        
        //static uint32_t test_timer=0;
        //if(millis()-test_timer>10000){
        //    dump_config();
        //    test_timer=millis();
        //}
        
        // обработка сенсорной кнопки
        if(sens_delay) {
            uint32_t _now=millis();
            if(_now-sens_timer>sens_delay+gap_timer){ // между нажатиями на кнопку должна быть пауза
               sens_delay=0; 
            } else if (_now-sens_timer>sens_delay){ // истекло время нажатия
               if(this->sens_pin_state==false){
                  this->sens_pin->digital_write(true);
                  this->sens_pin_state=true;
                  ESP_LOGD(TAG,"%d Release sensor LEFT",millis());
               }
               if(this->sens_pin2!=nullptr){
                  if(this->sens_pin2_state==false){
                     this->sens_pin2->digital_write(true);
                     this->sens_pin2_state=true;
                     ESP_LOGD(TAG,"%d Release sensor RIGHT",millis());
                  }
               }
            }               
        } 
        
        // обработка дисплея
        static uint32_t led_read_timer=millis();
        if(millis()-led_read_timer>200){ // проверяем дисплей 5 раз в секунду
            if(getStrFromLed() || millis()-chScrTimer>5000){ // показания дисплея изменились
                ESP_LOGV(TAG,"Display: %s",this->scr);
                // детектирование текущего режима работы
                bool waterOk=now_water; // для контроля сенсора наличия воды
                device_op operate=now_operate; //для контроля текущего режима
                if(scr[0]>='0' && scr[0]<='5' && scr[1]>='0' && scr[1]<='9'){ // показывает температуру или задержку слипа
                    uint8_t temp=(scr[0]-'0')*10+(scr[1]-'0');
                    waterOk=true;
                    if(temp>12){ //значение меньше, скорее всего SLEEP
                        if(main_temp!=nullptr && now_temperature!=temp){//датчик подключен и температура изменилась
                            ESP_LOGV(TAG,"Main temperature: %u",temp);
                            main_temp->publish_state(temp);
                            now_temperature=temp; // уравниваем значение
                        }
                    } else {
                        manual=true; // раз нажали кнопку слипа - перешли в ручное управление
                        ESP_LOGD(TAG,"Go to manual mode");
                        op_mode->publish_state(get_str_from_op_manual(operate));
                    }
                    if(now_operate==doERROR){ // восстановить индикацию режима работы
                        operate=save_operate;
                    }
                    if(save_operate==doIDLE || save_operate==doUNDEF){
                        save_operate=doMID;
                    }
                } else if(scr[0]=='E' && scr[1]=='1'){ // нет воды в резервуаре
                    waterOk=false; // воды нет
                    operate=doERROR; // ошибка
                } else if(scr[0]=='_' && scr[1]=='_'){ // слабое распыление
                    operate=doLOW; // слабое распыление
                } else if(scr[0]=='=' && scr[1]=='='){ // среднее распыление
                    operate=doMID; // среднее распыление
                } else if(scr[0]=='#' && scr[1]=='#'){ // сильное распыление
                    operate=doHIGH; // сильное распыление
                } else if(scr[0]==' ' && scr[1]==' ' &&  millis()-chScrTimer>5000 && operate!=doIDLE){ // выключено
                    operate=doIDLE; // простой
                    if(target_set!=dsOFF){ // не ручной режим
                        manual=true;
                        ESP_LOGD(TAG,"Set to OFF in manual mode");
                    }
                }
                chScrTimer=millis(); // таймштамп последнего изменения показаний дисплея
                if(water_ok!=nullptr && waterOk!=now_water){ // обслуживание сенсора наличия воды
                    water_ok->publish_state(waterOk);
                    now_water=waterOk;
                }         
                if(op_mode!=nullptr && now_operate!=operate){ // обслуживание сенсора текущего режима
                    if(operate == doERROR && /*now_operate!=doIDLE &&*/ now_operate!=doUNDEF){
                        save_operate=now_operate; // запоминаем для восстановления после ошибки
                    }
                    if(need_new_set==false){ // переключение внешним воздействием на кнопки
                        if(operate==doHIGH || operate==doMID || operate==doLOW){
                            manual=true;
                            ESP_LOGD(TAG,"Set to manual mode");
                        }
                    }
                    if(manual){
                       if(operate==doIDLE){ // тут отключение вручную
                          this->mode_select->publish_state(get_str_from_set(dsOFF));
                       } else if(operate==doLOW){
                          this->mode_select->publish_state(get_str_from_set(dsLOW));
                       } else if(operate==doMID){
                          this->mode_select->publish_state(get_str_from_set(dsMID));
                       } else if(operate==doHIGH){
                          this->mode_select->publish_state(get_str_from_set(dsHIGH));
                       }
                       op_mode->publish_state(get_str_from_op_manual(operate));
                    } else {
                       op_mode->publish_state(get_str_from_op(operate));
                    }
                    if(now_operate!=operate){
                        now_operate=operate;
                        if((now_operate==doLOW  && target_set==dsLOW) ||
                           (now_operate==doMID  && target_set==dsMID) ||
                           (now_operate==doHIGH && target_set==dsHIGH) ||
                           (now_operate==doIDLE && target_set==dsOFF)){
                             need_new_set=false; // установили запрошенный режим, скинуть флаг усановки
                        }
                    }
                }                    
            }                
            led_read_timer=millis();
        }
        
        // процессинг установки запрошенного по сети режима
        if(sensOn(STATUS) && need_new_set){ // если кнопка свободна и установлен запрос нового режима
            if((now_operate==doLOW  && target_set==dsLOW) ||
               (now_operate==doMID  && target_set==dsMID) ||
               (now_operate==doHIGH && target_set==dsHIGH)||
               (now_operate==doIDLE && target_set==dsOFF)){
                  need_new_set=false; // установили запрошенный режим 
            } else  if(target_set==dsOFF){ // требуется выключение
                sensOn(PUSH); // запрашиваем выключение
            } else if (now_water) { // если есть вода
                sensOn(CLICK); // кликаем сенсором , перебираем режимы
            } else { // если нет воды, то ребуемый режим усановиь нельзя
                need_new_set=false;    
            }
        }

    }
    
    void update() override {
        this->change_process();
    }

friend class HumiF600PresetSelect;
friend class HumiF600TargetNumber;
  
};
 
//HumiF600 &cast(Component *c) { return *reinterpret_cast<HumiF600 *>(c); }
  
}  // namespace deerma_f600
}  // namespace esphome
  
#endif  