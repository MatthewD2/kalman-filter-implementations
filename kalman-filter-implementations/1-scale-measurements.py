#The Following Program is designed to estimate the exact weight of an object given several weight measurements
#The weight of the object is treated as a random variable

#Store weight data here
init_state = 100 #g (what google said half a cup of flour is)
init_state_error = 5 #g
weight_data = [102, 101, 103, 101, 102, 101, 101, 101, 101, 101] #lbs
process_noise_var = 0.0000001 #g^2
measurement_var =  9 #g^2

class ScaleKalmanFilter:
##### The 5 Kalman Filter Equations #####
#State Update Equation
    def stateUpdate(self, predicted_x_hat,curr_measurement,k_gain):
        x_hat = predicted_x_hat + k_gain*(curr_measurement - predicted_x_hat) #Optain the current state estimation
        return x_hat

#State Extrapolation 
    def stateExtrapolation(self, prev_x_hat):
        predicted_x_hat = prev_x_hat #We use a constant dynamic model, because the weight of the object on the scale should be constant
        return predicted_x_hat

#Covariance Extrapolation Equation
    def covarianceExtrapolation(self, prev_state_var, process_noise_var):
        curr_state_var = prev_state_var + process_noise_var #found through the state extrapolation equation, and then adding process noise
        return curr_state_var

#Kalman Gain Equation
    def kalmanGain(self, state_var, measurement_var):
        k_gain = state_var / (state_var + measurement_var)
        print(k_gain)
        return state_var

#Covariance Update Equation
    def covarianceUpdate(self, prev_state_var, k_gain):
        state_var = (1-k_gain)*prev_state_var
        return state_var


##### Kalman Filter Step Functions #####
    #Initialization step
    def Initialize(self, init_state, init_state_error):
        init_state_var = init_state_error**2
        return init_state, init_state_var

    #Measure Step
    def Measure(self, state_measurement, measurement_var):
        return state_measurement, measurement_var 

    #Update Step
    def Update(self, curr_state_prediction, curr_state_var_prediction, curr_state_measurement, curr_measurement_var):
        k_gain = ScaleKalmanFilter.kalmanGain(self, curr_state_var_prediction, curr_measurement_var)
        curr_state_estimate = ScaleKalmanFilter.stateUpdate(self, curr_state_prediction,curr_state_measurement,k_gain)
        curr_state_var = ScaleKalmanFilter.covarianceUpdate(self, curr_state_var_prediction, k_gain)
        return curr_state_estimate, curr_state_var

    #Predict Step
    def Predict(self, curr_state_estimate, curr_state_var, curr_measurement, curr_measurement_var, process_noise_var):
        #if predicting during the initialization step, just return the current state estimate and current state variance
        if (curr_measurement == 0 and curr_measurement_var == 0):
            return curr_state_estimate, curr_state_var
        next_state_prediction = ScaleKalmanFilter.stateExtrapolation(curr_state_estimate, curr_measurement)
        next_var_prediction = ScaleKalmanFilter.covarianceExtrapolation(curr_state_var, curr_measurement_var, process_noise_var)
        return next_state_prediction, next_var_prediction
    

##### Driver Function #####
#define main()
def main(init_state, init_state_error, scale_measurements, scale_measurement_var, process_noise_var):
    measured_object = ScaleKalmanFilter()
    curr_state_prediction = measured_object.Predict(init_state, init_state_error, 0, 0, process_noise_var) #Step 0 Initialization
    for curr_measurement in scale_measurements:
        curr_measurement_data = measured_object.Measure(curr_measurement, scale_measurement_var) #Step 1 Measure
        curr_state_estimate = measured_object.Update(curr_state_prediction[0], curr_state_prediction[1], 
                                                     curr_measurement_data[0], curr_measurement_data[1]) #Step 2 Update
        print(str(curr_state_estimate) + '\n') #Output: The current state estimate and current state estimate variance
        curr_state_prediction = measured_object.Predict(curr_state_estimate[0],curr_state_estimate[1],
                                                        curr_measurement_data[0],curr_measurement_data[1], process_noise_var) #Step 3 Predict

#Run Driver to initiate Kalman Filter Program
main(init_state, init_state_error, weight_data, process_noise_var, measurement_var)

#Shoutout to KalmanFilter.net for teaching me a lot of the information used to create this KalmanFilter