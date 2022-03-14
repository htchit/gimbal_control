class MiniPID
{
public:
	void Init(double, double, double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setRate(double);
	void setILim(double);
	void setOLim(double);
	double run(double, double);

private:
	double P;
	double I;
	double D;

	double Rate;
	double ILim;
	double OLim;

	double fb;
	double target;
	double exp;

	double err_sum = 0;
	double err_last = 0;
};

void MiniPID::Init(double P, double I, double D, double Rate, double ILim, double OLim)
{
	setP(P);
	setI(I);
	setD(D);
	setRate(Rate);
	setILim(ILim);
	setOLim(OLim);
}
// ?

double MiniPID::run(double target, double fb)
{
	double err = target - fb;
	// Kp
	//  err
	// Ki
	this->err_sum += (err / Rate);

	if (this->err_sum >= this->ILim)
	{
		this->err_sum = this->ILim;
	}
	else if (this->err_sum <= -this->ILim)
	{
		this->err_sum = -this->ILim;
	}

	// kd
	double err_rate = 0;
	// init_define err_rate count.
	err_rate = (err - this->err_last) * this->Rate; // err_rate init?

	this->err_last = err;
	exp = P * err + I * this->err_sum + D * err_rate;
	if (this->exp >= this->OLim)
	{
		this->exp = this->OLim;
	}
	else if (this->exp <= -this->OLim)
	{
		this->exp = -this->OLim;
	}

	return exp;
}

void MiniPID::setP(double p)
{
	this->P = p;
}

void MiniPID::setI(double i)
{
	this->I = i;
}

void MiniPID::setD(double d)
{
	this->D = d;
}

void MiniPID::setRate(double rate)
{
	this->Rate = rate;
}

void MiniPID::setILim(double ilim)
{
	this->ILim = ilim;
}

void MiniPID::setOLim(double olim)
{
	this->OLim = olim;
}
