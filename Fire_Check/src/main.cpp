
#include <Arduino.h>
#include <U8x8lib.h>  
#include <Wire.h>
#include <ESP_Mail_Client.h>
#include <WiFi.h>

#include <Fire_monitoring_inferencing.h>


#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
/* wifi ssid和密码 */
#define WIFI_SSID "Wifi_ssid"
#define WIFI_PASSWORD "Wifi_password"

/* qq 邮箱的 smtp 地址和端口号 */
#define SMTP_HOST "smtp.qq.com"
#define SMTP_PORT 465

/* 
你的邮箱和授权码 授权码是QQ邮箱推出的，用于登录第三方客户端的专用密码。 
如何获取授权码 https://service.mail.qq.com/cgi-bin/help?subtype=1&&id=28&&no=1001256
*/
#define AUTHOR_EMAIL "your@qq.com"
#define AUTHOR_PASSWORD "bchbirabbsmjeaeg"
/* 接收人邮箱地址 email*/
#define RECIPIENT_EMAIL "XXXX@outlook.com"

/* 定义 smtp session 对象*/
SMTPSession smtp;

/* 获取邮件发送状态的回调函数 */
void getSmtpStatusCallback(SMTP_Status status);

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);  //设置oled大屏的参数





// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

#define CAMERA_MODEL_XIAO_ESP32S3  // Has PSRAM

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13
#define Buzzer 1
#define LED_GPIO_NUM 21


/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf;  //points to the output of the capture

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12,  //0-63 lower number means higher quality
  .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/**
* @brief      Arduino setup function
*/

unsigned long previousMillis = 0;  // 上一次计时的时间
unsigned long interval = 100;  // 时间间隔，1秒
int printCount_other = 0;  // 已经打印的次数
int printCount_mug = 0;  // 已经打印的次数
int printCount_scissors = 0; 

const int targetCount = 5;  // 目标打印的次数 


void setup() {

  pinMode(Buzzer,OUTPUT);
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  //pinMode(D0, OUTPUT);
  //pinMode(D7, OUTPUT);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("连接 Wifi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  Serial.println("WiFi 连接成功.");
  Serial.println("IP 地址: ");
  Serial.println(WiFi.localIP());
  Serial.println();


  //comment out the below line to start inference immediately after upload
  //while (!Serial);
  Serial.println("Edge Impulse Inferencing Demo");
  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
  } else {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

void loop() {

  // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);




#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  bool bb_found = result.bounding_boxes[0].value > 0;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
    auto bb = result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }
    ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
  }
  if (!bb_found) {
    ei_printf("    No objects found\n");
  }
#else

  int pred_index = 0;
  float pred_value = 0;

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label,
              result.classification[ix].value);
    if (result.classification[ix].value > pred_value) {
      pred_index = ix;
      pred_value = result.classification[ix].value;
    }
//此处是判断推荐类别是0，置信率大于0.8时，输出推理的标签名和预测数值
    if ((pred_index == 0) && (pred_value > 0.8)) {
      
    
      
      printCount_other++; 
      digitalWrite(Buzzer, LOW);
      //Serial.print("other:");
      //Serial.println(printCount_other);
        /* smtp开启debug，debug信息输出到串口 */
        smtp.debug(1);

        /* 注册回调函数，获取邮件发送状态 */
        smtp.callback(getSmtpStatusCallback);


        ESP_Mail_Session session;

        /* 设置smtp 相关参数， host, port等 */
        session.server.host_name = SMTP_HOST;
        session.server.port = SMTP_PORT;
        session.login.email = AUTHOR_EMAIL;
        session.login.password = AUTHOR_PASSWORD;
        session.login.user_domain = "";

        /* 定义smtp message消息类 */
        SMTP_Message message;

        /* 定义邮件消息类的名称，发件人，标题和添加收件人 */
        message.sender.name = "Fire_Warning!";
        message.sender.email = AUTHOR_EMAIL;
        message.subject = "Fire_check_sencor";
        message.addRecipient("Sara", RECIPIENT_EMAIL);

        /* 设置message html 格式和内容*/
        String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Battery car is on fire, please check immediately or dial 119</h1><p>Sent from Firechecker</p></div>";
        message.html.content = htmlMsg.c_str();
        message.html.content = htmlMsg.c_str();
        message.text.charSet = "us-ascii";
        message.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;


        /* 连接smtp服务器 */
        if (!smtp.connect(&session))
          return;
        if (!MailClient.sendMail(&smtp, &message))
          Serial.println("发送邮件失败");
        u8x8.clear();
        u8x8.setCursor(2, 2);
        u8x8.print("other:");
        u8x8.setCursor(8, 2);
        u8x8.print(printCount_other);

    } else if ((pred_index == 1) && (pred_value > 0.5)) {
      
      
      digitalWrite(Buzzer, HIGH);
      printCount_mug++;
      //Serial.print("scissors:");
      //Serial.println(printCount_scissors);
      u8x8.clear();
      u8x8.setCursor(2, 2);
      u8x8.print("mug:");
      u8x8.setCursor(7, 2);
      u8x8.print(printCount_mug);
     
    } else if ((pred_index == 2) && (pred_value > 0.5)) { 
      digitalWrite(Buzzer, HIGH);
      printCount_scissors++;
      //Serial.print("mug:");
      //Serial.println(printCount_mug);
      u8x8.clear();
      u8x8.setCursor(2, 2);
      u8x8.print("scissors:");
      u8x8.setCursor(12, 2);
      u8x8.print(printCount_scissors);
    } 
    else {
      //nothing to do.
    }
  
    }

  unsigned long currentMillis = millis();  // 当前时间，从开启Arduino开始计时
  if (currentMillis - previousMillis >= interval) {  // 判断是否到达规定的时间间隔
    previousMillis = currentMillis;  // 记录这一次进入执行代码块的时间
    /*if (printCount_other > targetCount) {  // 判断是否达到打印目标次数
      
      Serial.println("指向其他");
      //myServo.write(45); // 将引脚3设置为高电平
      //delay(5000);  // 等待5秒钟
      myServo.write(0);  // 
      printCount_other = 0; // 将printCount置零
    }*/
    if (printCount_mug > targetCount) {  // 判断是否达到打印目标次数
      delay(5000);  // 等待5秒钟
      printCount_mug = 0; // 将printCount置零
    }
    if (printCount_scissors > targetCount) {  // 判断是否达到打印目标次数
      delay(5000);  // 等待5秒钟
      printCount_scissors = 0; // 将printCount置零
    }
  }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


  free(snapshot_buf);

  }


/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

  if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_awb_gain(s, 1);
#endif

  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

  //deinitialize the camera
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;

  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  }


  return true;
}



#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
/* 获取发送状态的回调函数 */
void getSmtpStatusCallback(SMTP_Status status){
  /* 输出邮件发送状态信息 */
  Serial.println(status.info());

  /*状态获取成功，打印状态信息 */
  if (status.success()){
    Serial.println("----------------");
    
    ESP_MAIL_PRINTF("邮件发送成功个数: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("邮件发送失败个数: %d\n", status.failedCount());
    
    Serial.println("----------------\n");
    struct tm dt;

    for (size_t i = 0; i < smtp.sendingResult.size(); i++){
      /* 依次获取发送邮件状态 */
      SMTP_Result result = smtp.sendingResult.getItem(i);
      time_t ts = (time_t)result.timestamp;
      localtime_r(&ts, &dt);
      ESP_MAIL_PRINTF("收件人: %s邮件发送状态信息\n", result.recipients);
      ESP_MAIL_PRINTF("状态: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("发送时间: %d/%d/%d %d:%d:%d\n", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
      ESP_MAIL_PRINTF("邮件标题: %s\n", result.subject);
    }
    
    Serial.println("----------------\n");
  }
}
