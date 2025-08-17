/*
 * 
 * Versao com uma implementacao do RR com prioridade - 
 * Maior Valor, maior prioridade
 *    01/07/2024 
 *
*/

#include <avr/io.h>
#include <Arduino.h>
#include <TimerOne.h>
/*
*
* Vari veis do Kernel
*
*/
/*Define intervalo das interrup  es de Clock 
 * Tabela de Interrup  es :
 * 1       ClkT = 1   segundo
 * 0.1     ClkT = 100 milissegundos
 * 0.01    Clkt = 10  milissegundos
 * 0.001   Clkt = 1   milissegundo
 * 0.0001  ClkT = 100 microssegundos
 * 0.00001 ClkT = 10  microssegundos
*/
#define ClkT 2 //equivale a 2 segundos

#define Slice 1000000 //1 segundo
#define MaxNumberTask 5
#define NUM_TASKS 4
#define SizeTaskStack 128// Tamanho da pilha da tarefa
#define MAX_NKREAD_QUEUE 5 // N mero m ximo de threads esperando por leitura
#define MAX_NAME_LENGTH 30
unsigned int NumberTaskAdd=-1;
volatile int TaskRunning = 0;
char myName[MAX_NAME_LENGTH];
int  SchedulerAlgorithm ;

enum Scheduler{
  RR,
  RM,
  EDF
};
enum Taskstates{
  INITIAL,
  READY,
  RUNNING,
  DEAD,
  BLOCKED
};
typedef struct 
{
    int queue[MaxNumberTask];
    int tail;
    int head;
}ReadyList;
ReadyList ready_queue;

typedef struct 
{
  short count;
  int sem_queue[MaxNumberTask], tail, header,block;//block indica se tem uma task bloqueada no semaforo
  
}sem_t;


typedef struct {
    int tid; // ID da thread esperando pela leitura
    const char *format; // Formato da entrada esperado (similar ao scanf)
    void *var; // Argumentos onde os dados ser o armazenados
} NkReadQueueEntry;

NkReadQueueEntry nkreadQueue[MAX_NKREAD_QUEUE];
int nkreadQueueHead = 0;
int nkreadQueueTail = 0;
char serialInputBuffer[128]; // Buffer para armazenar a entrada da serial
int serialInputIndex = 0;

typedef struct
{
  int CallNumber;
  unsigned char *p0;
  unsigned char *p1;
  unsigned char *p2;
  unsigned char *p3;
}Parameters;
volatile Parameters kernelargs ;


typedef struct {
  int16_t Tid;
  const char *name;
  unsigned short Prio;
  unsigned int Time;
  unsigned short Join;
  unsigned short State;
  uint8_t Stack[SizeTaskStack]; // Vetor de pilha
  uint8_t* P; // Ponteiro de pilha
} TaskDescriptor;
TaskDescriptor Descriptors[MaxNumberTask]; // Array de descritores de tarefas
/* 
*
*Servicos do kernel
*
*/
enum sys_temCall{
  TASKCREATE,
  SEM_WAIT,
  SEM_POST,
  SEM_INIT,
  WRITELCDN,
  WRITELCDS,
  EXITTASK,
  SLEEP,
  MSLEEP,
  USLEEP,
  LIGALED,
  DESLIGALED,
  START, 
  TASKJOIN,
  SETMYNAME,
  GETMYNAME,
  NKPRINT,
  GETMYNUMBER,
  NKREAD,
   SEM_WAITB,
  SEM_POSTB,
  SEM_INITB,
};

/*************************************************************
*                                                            *
* Rotinas do kernel                                          *
*                                                            *
*                                                            *
*************************************************************/
void kernel() {
    switch(kernelargs.CallNumber){
    case TASKCREATE: 
      sys_taskcreate((int *)kernelargs.p0,(void(*)())kernelargs.p1,(int *)kernelargs.p2);
      break;
    case SEM_WAIT: 
     // Serial.println("SEMWAIT: ") ;
      sys_semwait((sem_t *)kernelargs.p0);
      break;
    case SEM_POST: 
      sys_sempost((sem_t *)kernelargs.p0);
      break;
    case SEM_INIT: 
      //Serial.println("SEMINIT: ") ;
      sys_seminit((sem_t *)kernelargs.p0,(int)kernelargs.p1);
      break;
       case SEM_WAITB: 
     // Serial.println("SEMWAIT: ") ;
      sys_semwaitB((sem_t *)kernelargs.p0);
      break;
    case SEM_POSTB: 
      sys_sempostB((sem_t *)kernelargs.p0);
      break;
    case SEM_INITB: 
      //Serial.println("SEMINIT: ") ;
      sys_seminitB((sem_t *)kernelargs.p0,(int)kernelargs.p1);
      break;
    case WRITELCDN: // NAO TEREMOS
      // LCDcomando((int)arg->p1);
      // LCDnum((int)arg->p0);
      break;
    case WRITELCDS: // NAO TEREMOS
      // LCDcomando((int)arg->p1);
      // LCDputs((char*)arg->p0);
      break;
    case EXITTASK: 
      sys_taskexit();
      break;
    case SLEEP: 
      sys_sleep((int)kernelargs.p0);
      break;
    case MSLEEP: 
      sys_msleep((int)kernelargs.p0);
      break;
    case USLEEP: 
      sys_usleep((int)kernelargs.p0);
      break;
    case LIGALED: 
      sys_ligaled();
      break;
    case DESLIGALED: 
      sys_desligaled();
      break;
    case START: 
      sys_start((int)kernelargs.p0);
      break;
    case TASKJOIN: // NAO TEREMOS
     // sys_taskjoin((int)arg->p0);
      break;
    case SETMYNAME: 
      sys_setmyname((const char *)kernelargs.p0);
      break;
    case GETMYNAME: 
      sys_getmyname((const char *)kernelargs.p0);
      break;  
    case NKPRINT: 
       sys_nkprint((char *)kernelargs.p0,(void *)kernelargs.p1);
       break;
    case GETMYNUMBER: 
       sys_getmynumber((int *)kernelargs.p0);
       break;
    case NKREAD: 
       sys_nkread((char *)kernelargs.p0,(void *)kernelargs.p1);
       break;
    default:
       break;
  }
}
/*
* Passa a executar a rotina do kernel com interrupcoes desabiitadas
*
*/
void callsvc(Parameters *args)
{
  noInterrupts();
  kernelargs = *args ;
  kernel() ;
  interrupts();
}
/*
*
*
*/
void saveContext(TaskDescriptor* task) {
    asm volatile (
        "push r0 \n\t"
        "in r0, __SREG__ \n\t"
        "cli \n\t"
        "push r0 \n\t"
        "push r1 \n\t"
        "clr r1 \n\t"
        "push r2 \n\t"
        "push r3 \n\t"
        "push r4 \n\t"
        "push r5 \n\t"
        "push r6 \n\t"
        "push r7 \n\t"
        "push r8 \n\t"
        "push r9 \n\t"
        "push r10 \n\t"
        "push r11 \n\t"
        "push r12 \n\t"
        "push r13 \n\t"
        "push r14 \n\t"
        "push r15 \n\t"
        "push r16 \n\t"
        "push r17 \n\t"
        "push r18 \n\t"
        "push r19 \n\t"
        "push r20 \n\t"
        "push r21 \n\t"
        "push r22 \n\t"
        "push r23 \n\t"
        "push r24 \n\t"
        "push r25 \n\t"
        "push r26 \n\t"
        "push r27 \n\t"
        "push r28 \n\t"
        "push r29 \n\t"
        "push r30 \n\t"
        "push r31 \n\t"
        "in %A0, __SP_L__ \n\t"
        "in %B0, __SP_H__ \n\t"
        : "=r" (task->P)
    );
}
void restoreContext(TaskDescriptor* task) {
    asm volatile (
        "out __SP_L__, %A0 \n\t"
        "out __SP_H__, %B0 \n\t"
        "pop r31 \n\t"
        "pop r30 \n\t"
        "pop r29 \n\t"
        "pop r28 \n\t"
        "pop r27 \n\t"
        "pop r26 \n\t"
        "pop r25 \n\t"
        "pop r24 \n\t"
        "pop r23 \n\t"
        "pop r22 \n\t"
        "pop r21 \n\t"
        "pop r20 \n\t"
        "pop r19 \n\t"
        "pop r18 \n\t"
        "pop r17 \n\t"
        "pop r16 \n\t"
        "pop r15 \n\t"
        "pop r14 \n\t"
        "pop r13 \n\t"
        "pop r12 \n\t"
        "pop r11 \n\t"
        "pop r10 \n\t"
        "pop r9 \n\t"
        "pop r8 \n\t"
        "pop r7 \n\t"
        "pop r6 \n\t"
        "pop r5 \n\t"
        "pop r4 \n\t"
        "pop r3 \n\t"
        "pop r2 \n\t"
        "pop r1 \n\t"
        "pop r0 \n\t"
        "out __SREG__, r0 \n\t"
        "pop r0 \n\t"
        : : "r" (task->P)
    );
}

void wakeUP() //acorda a task bloqueada a espera de passagem de tempo
{
  int i=1;
  for(i=1;i<=NUM_TASKS; i++)
  {
    //sleep
    if(Descriptors[i].Time>0)
    {
      Descriptors[i].Time--;
      if(Descriptors[i].Time <= 0 && Descriptors[i].State == BLOCKED)
      {
        Descriptors[i].State = READY;
        InsertReadyList(i) ; //tempo de espera se esgotou
      }
    }
  }
}

//Escalonador

/*
* Imprime a Ready List
* Usada para  testes
*/
 void printReadyList() {
    Serial.print("Ready list tasks: ");
    for (int i = 0; i < ready_queue.head; i++) {
        Serial.print(" Index:");
        Serial.print(ready_queue.queue[i]);
    }
    Serial.println(" ");
}


/*
* Insere a task no final da Ready List
*  sortReadyList() realizada na switchTask()
*/
void InsertReadyList(int id) {
  ready_queue.queue[ready_queue.head] = id;
  ready_queue.head++;
  
}


/*
*   
* Se a task atual n o   Idle (TaskRunning != 0), a task   removida da Ready List
* Caso n o esteja bloqueada, ela   reinserida no final da Ready List
* A remo  o   feita com o deslocamento para a esquerda 
* Chama a fun  o sortReadyList()
* Atualiza TaskRunning com a primeira task da Ready List (TaskRunning = ready_queue.queue[0])
* Se a Ready List estiver vazia, TaskRunning ser  0 (Idle) 
*
*/
void switchTask() {    
  saveContext(&Descriptors[TaskRunning]);

  if (TaskRunning != 0){
    for (int i = 0; i < ready_queue.head - 1; i++) {
      ready_queue.queue[i] = ready_queue.queue[i + 1];
    }
    ready_queue.head--;
    if (Descriptors[TaskRunning].State != BLOCKED)  {   
      InsertReadyList(TaskRunning);   
    }
  }
  sortReadyList();
  if (ready_queue.head > 0){
    TaskRunning = ready_queue.queue[0];
  } else {
    TaskRunning = 0;
  }
  
  Descriptors[TaskRunning].State = RUNNING;
  restoreContext(&Descriptors[TaskRunning]);
}

/*
*   
* Algoritmo Bubble Sort para a reordena o da Ready List
* O crit rio de ordena  o   a prioridade (Prio) definida para a task
* Menor valor n merico indica maior prioridade
*
*/
void sortReadyList() {
  for (int i = 0; i < ready_queue.head - 1; i++) {
      for (int j = 0; j < ready_queue.head - i - 1; j++) {
          if (Descriptors[ready_queue.queue[j]].Prio > Descriptors[ready_queue.queue[j + 1]].Prio) {
              int temp = ready_queue.queue[j];
              ready_queue.queue[j] = ready_queue.queue[j + 1];
              ready_queue.queue[j + 1] = temp;
          }
      }
  }
}

//Trata a interrupcao do Timer

void systemContext() { //Chamada pela interrupcao do Timer
  wakeUP();
  serialEvent();
  switchTask();
}

/*
*
* Idle Process - executa quando ready list vazia
*
*/
void idle() {
     while (1) {
     } ;
}
/*
*
* Rotinas do kernel - Sys Call
*
*/
void sys_taskcreate(int *tid, void (*taskFunction)(void), int priority) {

    NumberTaskAdd++ ;
    *tid = NumberTaskAdd;
    Descriptors[NumberTaskAdd].Tid=*tid;
    Descriptors[NumberTaskAdd].State=READY;
    Descriptors[NumberTaskAdd].Join=0;
    Descriptors[NumberTaskAdd].Time=0 ;
    Descriptors[NumberTaskAdd].Prio=priority;
    uint8_t* stack = Descriptors[*tid].Stack + SizeTaskStack - 1;
    Descriptors[*tid].P = stack;

    *(stack--) = ((uint16_t)taskFunction) & 0xFF; // PC low byte
    *(stack--) = ((uint16_t)taskFunction >> 8) & 0xFF; // PC high byte
    *(stack--) = 0x00; // R0
    *(stack--) = 0x80; // SREG with global interrupts enabled

    for (int i = 1; i < 32; i++) {
        *(stack--) = i; // Initialize all other registers with their number
    }

    Descriptors[*tid].P = stack;
}



void sys_start(int scheduler) {
    int i;
    SchedulerAlgorithm = scheduler;
    switch (SchedulerAlgorithm) {
        case RR:
            for (i = 1; i <= NumberTaskAdd; i++) {
                InsertReadyList(i);
            }
            sortReadyList();
            break;
        default:
            break;
    }
}

void sys_getmynumber(int *number)
{
  *number=Descriptors[TaskRunning].Tid ;
}

void sys_ligaled()
{
  PORTB = PORTB | 0x20;
}

void sys_desligaled()
{
  PORTB = PORTB & 0xDF;
}

void sys_setmyname(const char *name)
{
  Descriptors[TaskRunning].name=name;
}

void sys_getmyname(const char *name)
{
  strcpy(name, Descriptors[TaskRunning].name);
}

void sys_semwait(sem_t *semaforo)
{   
    semaforo->count--;
    if(semaforo->count < 0)
    {
      semaforo->sem_queue[semaforo->tail] = TaskRunning;
      Descriptors[TaskRunning].State = BLOCKED ;
      semaforo->tail++;
      if(semaforo->tail == MaxNumberTask-1) semaforo->tail = 0;
      switchTask();
    }
}

void sys_sempost(sem_t *semaforo)
{
     semaforo->count++;
     if(semaforo->count <= 0)
     {
       Descriptors[semaforo->sem_queue[semaforo->header]].State = READY;
       InsertReadyList(semaforo->sem_queue[semaforo->header]);
       semaforo->header++;
       if(semaforo->header == MaxNumberTask-1) semaforo->header = 0;
     }
}

void sys_seminit(sem_t *semaforo, int ValorInicial)
{
  semaforo->count = ValorInicial;
  semaforo->header = 0;
  semaforo->tail = 0;
  
}

void sys_semwaitB(sem_t *semaforo)
{
    if (semaforo->count == 1) {
        semaforo->count = 0; // Entra na região crítica
    } else {
        // Bloqueia a tarefa se count == 0
        semaforo->sem_queue[semaforo->tail] = TaskRunning;
        //sem_queue guarda o id da task bloqueada e o tail coloca a task no fim da fila
        //com isso a task é bloqueada
        Descriptors[TaskRunning].State = BLOCKED;
        semaforo->block=1;
        //tarefa bloqueada no vetor de descritores
        semaforo->tail++;
       if(semaforo->tail == MaxNumberTask-1) semaforo->tail = 0;
        switchTask(); // Troca de contexto, pois a tarefa esta bloqueada
    }
}

void sys_sempostB(sem_t *semaforo)
{
  //Verifica se há tarefas esperando no semáforo.
//A fila é circular.Se count for 0 e block for 1,a task é bloqueada e fica na lista de espera.
    if (semaforo->count==0 && semaforo->block==1) {
      
      
        // Há tarefas esperando
        //Recupera o ID da tarefa que está no início da fila de espera (posição header).
        int task = semaforo->sem_queue[semaforo->header];
        //Marca essa tarefa como pronta para executar (READY).
        //A função InsertReadyList(task) insere a tarefa na lista de tarefas aptas a executar (fila de prontos).
        Descriptors[task].State = READY;
        InsertReadyList(task);
        //Avança o ponteiro header na fila circular.
        //Se chegou ao fim do vetor, volta para o início.
        semaforo->header++;
        if(semaforo->header == MaxNumberTask-1) semaforo->header = 0;
    } else {
        // Ninguém esperando, libera o semáforo
        semaforo->count = 1;
    }
}

void sys_seminitB(sem_t *semaforo, int ValorInicial)
{
    if (ValorInicial != 0 && ValorInicial != 1)
        ValorInicial = 1; // Garante binário,valor inicial define o numero de threads q podem entrar na seção critica

    semaforo->count = ValorInicial;
    semaforo->header = 0;//aponta para a próxima tarefa que será desbloqueada quando o recurso for liberado.(INICIO DA FILA)
    semaforo->tail = 0;//aponta para onde a próxima tarefa bloqueada será inserida na fila.(FIM DA FILA)
    semaforo->block=0;
}
void sys_sleep(unsigned int segundo)
{
  //Descriptors[TaskRunning].Time = segundo/ClkT;
  Descriptors[TaskRunning].Time = (segundo*1000000)/Slice ;
  if(Descriptors[TaskRunning].Time > 0)
  {
    Descriptors[TaskRunning].State = BLOCKED;
    switchTask();
    
    //select() ;
  }
}
void sys_msleep(unsigned int mili)
{
  Descriptors[TaskRunning].Time = (mili/ClkT)/1000;
  if(Descriptors[TaskRunning].Time > 0)
  {
    Descriptors[TaskRunning].State = BLOCKED;
    switchTask();
  }
}

void sys_usleep(unsigned int micro)
{
  Descriptors[TaskRunning].Time = (micro/ClkT)/1000000;
  if(Descriptors[TaskRunning].Time > 0)
  {
    Descriptors[TaskRunning].State = BLOCKED;
    switchTask();
  }
}

/*
*  calcularPrecisao( float valor) chamada pela sys_nkprint
*/
static inline int calcularPrecisao( float valor)
{
  int PRECISAO_FLOAT_ARDUINO = 6;
  int precisao = 0;
  int valorInteiro = (int)valor;
  while(valorInteiro > 0)
  {
    valorInteiro = valorInteiro / 10;
    precisao++;
  }
  return PRECISAO_FLOAT_ARDUINO - precisao;
}
void sys_nkprint(char *fmt,void *number)
{
  int *auxint;
  float *auxfloat;
  char *auxchar;
  int size=0;
  int accuracy=1;
  
  while (*fmt)
  {
    switch(*fmt)
      {
        case '%':
          fmt++;
          switch(*fmt)
            {
              case '%':
                Serial.print(*fmt);
                break;
              case 'c':
                Serial.print((char *)number);
                break;
              case 's':
                sys_nkprint((char *)number,0);
                break;
              case 'd':
                auxint=number;
                Serial.print(*auxint);
                break;
              case 'f':
                auxfloat=number;
                int precisao = calcularPrecisao(*auxfloat);
                Serial.print(*auxfloat, precisao);
                break;
              // case '.':
              //   fmt++;
              //   while(*fmt != 'f')
              //   {
              //     size*=accuracy;
              //     size+= (*fmt - '0');
              //     accuracy*=10;
              //     fmt++;
              //   }
              //   auxfloat=number;
              //   // printfloat(*auxfloat,size);
              //   break;
              // case 'x':
              //   auxint=number;
              //   // printhexL(*auxint);
              //   break;
              // case 'X':
              //   auxint=number;
              //   // printhexU(*auxint);
              //   break;
              // case 'b':
              //   fmt++;
              //   switch(*fmt)
              //   {
              //     case 'b':
              //       size=8;
              //       break;
              //     case 'w':
              //       size=16;
              //       break;
              //     case 'd':
              //       size=32;
              //       break;
              //     default:
              //       fmt -= 2;
              //       size = 32;
              //       break;
              //   }
              //   auxint=number;
              //   // printbinary(*auxint, size);
              //   break;
              default:
                break;
            }
        break;
        case '\\':
          fmt++;
          if (*fmt == 'n')
          {
            Serial.print("\n");
          }
          else
          {
            Serial.print("\\");
          }
        break;
        default:
          Serial.print(*fmt);
        break;
      }
    fmt++;
  Serial.flush();
  delay(100) ;
  }
}

void sys_taskexit(void)
{
  Descriptors[TaskRunning].State=BLOCKED;
  switchTask();
}
void enqueueNkRead(int tid, const char *format, void *var) {
    nkreadQueue[nkreadQueueTail].tid = tid;
    nkreadQueue[nkreadQueueTail].format = format;
    nkreadQueue[nkreadQueueTail].var = var;
    nkreadQueueTail = (nkreadQueueTail + 1) % MAX_NKREAD_QUEUE;
}

NkReadQueueEntry dequeueNkRead() {
    NkReadQueueEntry entry = nkreadQueue[nkreadQueueHead];
    nkreadQueueHead = (nkreadQueueHead + 1) % MAX_NKREAD_QUEUE;
    return entry;
}

void sys_nkread(const char *format, void *var) {
  // Adicionar a thread atual na fila de leitura
  enqueueNkRead(Descriptors[TaskRunning].Tid, format, var);
  // Bloquear a thread atual
  Descriptors[TaskRunning].State = BLOCKED;
  switchTask();
}

float stringToFloat(const char* str) {
    float result = 0.0;
    float factor = 1.0;

    if (*str == '-') {
        str++;
        factor = -1.0;
    }

    // Parte inteira
    for (; *str >= '0' && *str <= '9'; str++) {
        result = result * 10.0 + (*str - '0');
    }

    // Parte fracion?ria
    if (*str == '.') {
        float fraction = 0.1;
        str++;
        for (; *str >= '0' && *str <= '9'; str++) {
            result += (*str - '0') * fraction;
            fraction *= 0.1;
        }
    }

    return result * factor;
}

void serialEvent() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            serialInputBuffer[serialInputIndex] = '\0';  // Termina a string
            serialInputIndex = 0;  // Reinicia o ?ndice

            // Desbloquear a thread que esta esperando por entrada
            if (nkreadQueueHead != nkreadQueueTail) {
                NkReadQueueEntry entry = dequeueNkRead();
                if (strcmp(entry.format, "%f") == 0) {
                    // Para float, usar nossa fun??o auxiliar
                    *(float *)(entry.var) = stringToFloat(serialInputBuffer);
                } else {
                    // Interpretar a entrada de acordo com o formato fornecido
                    sscanf(serialInputBuffer, entry.format, entry.var);
                }
                Descriptors[entry.tid].State = READY;
            }
        } else {
            if (serialInputIndex < 127) {
                serialInputBuffer[serialInputIndex++] = c;
            }
        }
    }
}


/*************************************************************
*                                                            *
* Chamadas de Sistema a N vel de usu rio                     *
*           User Call                                        *
*                                                            *
*************************************************************/

void taskcreate(int *ID,void (*funcao)(), int *Priority) //parametros armazenados em R0 e R1 na chamada
{
  Parameters arg;
  arg.CallNumber=TASKCREATE;
  arg.p0=(unsigned char *)ID;
  arg.p1=(unsigned char *)funcao;
  arg.p2=(unsigned char *)Priority;
  callsvc(&arg);
}
void start(int scheduler)
{
  Parameters arg;
  arg.CallNumber=START;
  arg.p0=(unsigned char *)scheduler;
  callsvc(&arg);
}
void semwait(sem_t *semaforo)
{
  Parameters arg;
  arg.CallNumber=SEM_WAIT;
  arg.p0=(unsigned char *)semaforo;
  callsvc(&arg);
}

void sempost(sem_t *semaforo)
{
  Parameters arg;
  arg.CallNumber=SEM_POST;
  arg.p0=(unsigned char *)semaforo;
  callsvc(&arg);
}

void seminit(sem_t *semaforo, int ValorInicial)
{
  Parameters arg;
  arg.CallNumber=SEM_INIT;
  arg.p0=(unsigned char *)semaforo;
  arg.p1=(unsigned char *)ValorInicial;
  callsvc(&arg);
}
void semwaitB(sem_t *semaforo)
{
  Parameters arg;
  arg.CallNumber=SEM_WAITB;
  arg.p0=(unsigned char *)semaforo;
  callsvc(&arg);
}

void sempostB(sem_t *semaforo)
{
  Parameters arg;
  arg.CallNumber=SEM_POSTB;
  arg.p0=(unsigned char *)semaforo;
  callsvc(&arg);
}

void seminitB(sem_t *semaforo, int ValorInicial)
{
  Parameters arg;
  arg.CallNumber=SEM_INITB;
  arg.p0=(unsigned char *)semaforo;
  arg.p1=(unsigned char *)ValorInicial;
  callsvc(&arg);
}
void setmyname(const char *name)
{
  Parameters arg;
  arg.CallNumber=SETMYNAME;
  arg.p0=(unsigned char *)name;
  callsvc(&arg);
}
void getmynumber(int *number)
{
  Parameters arg;
  arg.CallNumber=GETMYNUMBER;
  arg.p0=(unsigned char *)number;
  callsvc(&arg);
}
void getmyname(const char *name)
{
  Parameters arg;
  arg.CallNumber=GETMYNAME;
  arg.p0=(unsigned char *)name;
  callsvc(&arg);
}
void sleep(int time)
{
  Parameters arg;
  arg.CallNumber=SLEEP;
  arg.p0=(unsigned char *)time;
  callsvc(&arg);
}
void msleep(int time)
{  
  Parameters arg;
  arg.CallNumber=SLEEP;
  arg.CallNumber=MSLEEP;
  arg.p0=(unsigned char *)time;
  callsvc(&arg);
}

void usleep(int time)
{
  Parameters arg;
  arg.CallNumber=USLEEP;
  arg.p0=(unsigned char *)time;
  callsvc(&arg);
}
void taskexit(void)
{
  Parameters arg;
  arg.CallNumber=EXITTASK;
  callsvc(&arg);
}

void ligaled(void)
{
  Parameters arg;
  arg.CallNumber=LIGALED;
  callsvc(&arg);
}

void desligaled(void)
{
  Parameters arg;
  arg.CallNumber=DESLIGALED;
  callsvc(&arg);
}

void nkprint(char *fmt,void *number)
{
  Parameters arg;
  arg.CallNumber=NKPRINT;
  arg.p0=(unsigned char *)fmt;
  arg.p1=(unsigned char *)number;
  callsvc(&arg);
}
void nkread(const char *format, void *var) {
    Parameters arg;
    arg.CallNumber = NKREAD;
    arg.p0 = (unsigned char *)format;
    arg.p1 = (unsigned char *)var;
    callsvc(&arg);
}
/*************************************************************
*                                                            *
*                   Programa do  usu rio                     *
*                       - Aplica  o -                        *
*                                                            *
*************************************************************/


volatile int16_t tid0, tid1, tid2, tid3, tid4;
int i, j ;
sem_t sem1, sem2,sem3;

//sem_t s0;
//sem_t s1;
//sem_t s2;
//sem_t s3;


void p0() {     
  static int number0;
  getmynumber(&number0);  
  while (1) {
    semwaitB(&sem1);
    printReadyList();
    //nkprint("P0 will sleep: ", 0);
    sleep(1);
    nkprint("P0 running: ", 0);
    sempostB(&sem2);
  }
}

void p1() {
  static int number1 ;
  getmynumber(&number1);
  while (1) {
    semwaitB(&sem2);
    printReadyList();
    //nkprint("P1 will sleep: ", 0);
    sleep(1);
    nkprint("P1 running: ", 0);
    sempostB(&sem3);
  }
}

void p2() {
  static int number2;
  getmynumber(&number2);
  while (1) {
    semwaitB(&sem3);
    printReadyList();
    //nkprint("P2 will sleep: ", 0);
    sleep(1);
    nkprint("P2 running: ", 0);
    sempostB(&sem1);
  }
}

/*************************************************************
*                                                            *
*               Setup e cria  o das Tasks                    *
*                                                            *
*                                                            *
*************************************************************/


void setup() {
    Serial.begin(9600) ;
    nkprint("FakeOS \n", 0) ;
    nkprint("Versao 0.0 \n", 0) ;
  
    //seminit(&s0, 1);
    //seminit(&s1, 0);
    //seminit(&s2, 0);
    //seminit(&s3, 0);
   
    seminitB(&sem1,1);
    seminitB(&sem2,0);
    seminitB(&sem3,0);
    taskcreate(&tid0,idle,0);
    taskcreate(&tid1,p0,0);
    taskcreate(&tid2,p1,1);
    taskcreate(&tid3,p2,2);
    start (RR) ; //coloca as tasks na fila

  noInterrupts(); 
  Timer1.initialize(Slice);
 // Timer1.initialize(ClkT*1000000);
  Timer1.attachInterrupt(systemContext);
  restoreContext(&Descriptors[0]); //coloca a task idle para rodar
}

void loop() {
  while (1);
}
