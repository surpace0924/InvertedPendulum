void drive(int motor_val, float output_val, bool is_brk)
{
    if (output_val > 1)
    {
        output_val = 1;
    }
    else if (output_val < -1)
    {
        output_val = -1;
    }

    int dir = (output_val > 0) ? 0 : 1;
    int val = abs((int)(output_val * 255));
    int sendVal = abs((int)(output_val * 100));

    
    mySerial.write(dir << 7 | map(sendVal, 0, 100, 30, 100));

    Serial.println(output_val*100);

    if (is_brk)
    {
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(6, HIGH);
        digitalWrite(7, HIGH);
    }
    else
    {
        if (output_val > 0)
        {
            digitalWrite(2, LOW);
            analogWrite(3, val);
            digitalWrite(7, LOW);
            analogWrite(6, val);
        }
        else
        {
            digitalWrite(3, LOW);
            analogWrite(2, val);
            digitalWrite(6, LOW);
            analogWrite(7, val);
        }
    }

    // Serial.println(output_val * 255);
}
