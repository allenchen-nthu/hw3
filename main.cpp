#include "mbed.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"
#include "uLCD_4DGL.h"
#include "mbed_events.h"
#include <iostream>
#include "magic_wand_model_data.h"
#include "accelerometer_handler.h"
#include "confi.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include <cmath>

using namespace std::chrono;
using namespace std;

InterruptIn button(USER_BUTTON);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
uLCD_4DGL uLCD(D1, D0, D2);

int led = 0;
int angle_list[5] = {30, 35, 40, 45, 50};
int mode = 0;
int angle_index;
int public_ctrl = 0;
int keep_in_loop = 1;

Thread thread1;
Thread thread2;
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue2(32 * EVENTS_EVENT_SIZE);
EventQueue queue3(32 * EVENTS_EVENT_SIZE);

WiFiInterface *wifi;
volatile bool closed = false;

char* topic1 = "Mbed1";
char* topic2 = "Mbed2";
char* topic;

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;

void readRPCCommand();
void messageArrived(MQTT::MessageData& md);
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
void connectWIFI();

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

void flip(Arguments *in, Reply *out) {mode = !mode;}
void flip_2() {mode = !mode;}

BufferedSerial pc(USBTX, USBRX);

void start_tiltDetective_threat(Arguments *in, Reply *out);
void tiltDetective();

void start_gesture_threat(Arguments *in, Reply *out);
void gesture();//Arguments *in, Reply *out);

RPCFunction RPCgesture_IU(&start_gesture_threat, "gesture");
RPCFunction changeMode(&flip, "broker");
RPCFunction tiltMode(&start_tiltDetective_threat, "tilt");

void uLCD_print(int angle);

int main() {

	BSP_ACCELERO_Init();
	thread1.start(callback(&queue, &EventQueue::dispatch_forever));
	thread2.start(callback(&queue2, &EventQueue::dispatch_forever));
	queue3.call(&connectWIFI);
  /****************************publish mqtt**************************************/

  	wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.68";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
     data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic1, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    if (client.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    cout << "wait for python\n";
	  mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    button.rise(mqtt_queue.event(&publish_message, &client));
    /***************************end of publish mqtt*******************************/
    readRPCCommand();

}

void readRPCCommand(){
    char buf[256], outbuf[256];
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        printf("%s\r\n", buf);
        RPC::call(buf, outbuf);
    }
}

int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < confi.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

void gesture() {
  led1 = 1;
  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return ;//-1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != confi.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return ;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return ;
  }

  error_reporter->Report("start angle selection\n");

  while (mode == 0) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);
    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // us gesture index to control the index

    if(gesture_index == 0) {
      if(angle_index < 4)angle_index++; else angle_index = 0;
      uLCD_print(angle_list[angle_index]);
	  cout << "index :" << angle_index << endl;
    } else if (gesture_index == 1) {
	if(angle_index > 0)angle_index--; else angle_index = 4;
      //angle_index--;
      uLCD_print(angle_list[angle_index]);
	  cout << "index :" << angle_index << endl;
    }
    
    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(confi.output_message[gesture_index]);
    }
  }
  cout << " confirm index :" << angle_list[angle_index % 5] << endl;
  led1 = 0;
}

void uLCD_print(int angle)
{
    uLCD.background_color(WHITE);
    uLCD.textbackground_color(WHITE);
    uLCD.cls();

    uLCD.text_width(4); //4X size text
    uLCD.text_height(4);
    uLCD.color(GREEN);
    uLCD.locate(1,2);
    uLCD.cls();
    uLCD.text_width(4); //4X size text
    uLCD.text_height(4);
    
    uLCD.locate(1,2);
    uLCD.printf("%2d",angle);
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {

    MQTT::Message message;
    char buff[100];

    if(mode == 0) {
      topic = topic1;
      printf("topic1\n");
      sprintf(buff, "%d\r\n", angle_list[angle_index]);
    } else {
      topic = topic2;
      printf("topic2\n");
      sprintf(buff, "%d\r\n", angle_list[angle_index]);
    }

    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    
    int rc = client->publish(topic, message);

    ThisThread::sleep_for(100ms);
}

void connectWIFI() {


	while(1) {ThisThread::sleep_for(500ms);}
}


void start_gesture_threat (Arguments *in, Reply *out) {
    queue.call(&gesture);
}

void start_tiltDetective_threat (Arguments *in, Reply *out) {
    queue2.call(&tiltDetective);
}

void tiltDetective() {
    int16_t pDataXYZ_init[3] = {0};
    int16_t pDataXYZ[3] = {0};
    double mag_A;
    double mag_B;
    double cos;
    double rad_det;
    double angle_det;
   
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client2(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.68";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return ;//-1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed2\0";

    if ((rc = client2.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client2.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    printf("enter angle detection mode\r\n");
    printf("Place the mbed on tablefor reference\r\n");
    
    BSP_ACCELERO_AccGetXYZ(pDataXYZ_init);
    printf("reference acceleration vector: %d, %d, %d\r\n", pDataXYZ_init[0], pDataXYZ_init[1], pDataXYZ_init[2]);

    led2 = 1;
                // tile Angle_Detection mode
    int n = 1;
    while (mode) {
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        printf("Data of X, Y, Z : %d %d %d\r\n",pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        mag_A = sqrt(pDataXYZ_init[0]*pDataXYZ_init[0] + pDataXYZ_init[1]*pDataXYZ_init[1] + pDataXYZ_init[2]*pDataXYZ_init[2]);
        mag_B = sqrt(pDataXYZ[0]*pDataXYZ[0] + pDataXYZ[1]*pDataXYZ[1] + pDataXYZ[2]*pDataXYZ[2]);
        cos = ((pDataXYZ_init[0]*pDataXYZ[0] + pDataXYZ_init[1]*pDataXYZ[1] + pDataXYZ_init[2]*pDataXYZ[2])/(mag_A)/(mag_B));
        rad_det = acos(cos);
        angle_det = 180.0 * rad_det/3.1415926;
        int f = 1;
        printf("Angle is %f\r\n", angle_det);
        uLCD_print(angle_det);
          if ((angle_det > angle_list[angle_index]) && (n == 1) && (f == 1)) {
            printf("over tilt 1 time\n");
            ThisThread::sleep_for(1000ms);
            n++;
            f = 0;
          }
          if ((angle_det > angle_list[angle_index]) && (n == 2) && (f == 1)) {
            printf("over tilt 2 times\n");
            ThisThread::sleep_for(1000ms);
            n++;
            f = 0;
          }
          if ((angle_det > angle_list[angle_index]) && (n == 3) && (f == 1)) {
            mqtt_queue.call(&publish_message, &client2);
            printf("over tilting\n");
            led2 = 0;
          }
          ThisThread::sleep_for(1000ms);
    }
}