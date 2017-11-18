float a = 0.5;
float b = 1.5;
float c = 2.5;

void setup() {
  // put your setup code here, to run once:
}

void print_three_numbers(float a, float b, float c){
  Serial.print(a);
  Serial.print("\t");
  Serial.print(b);
  Serial.print("\t");
  Serial.println(c);
}

void loop() {
  // put your main code here, to run repeatedly:
  print_three_numbers(a, b, c);

  delay(100);
}
