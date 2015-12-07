class AvoidNaN{
    public:
        AvoidNaN();
        ~AvoidNaN();
        float ReplaceNaN(float Input);
    private:
        float PreOut;
}

class PID
{
 public:
 	PID();
    ~PID();

	float ComputePID(float Desired, float Meassured, float derivative, float Kp, float Ki, float Kd, float Dt);

  private:
   float integral;
   float PreOut;
};
