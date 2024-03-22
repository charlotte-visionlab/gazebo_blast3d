

/* 
 * File:   common.h
 * Author: arwillis
 *
 * Created on March 22, 2024, 8:31 AM
 */

#ifndef COMMON_H
#define COMMON_H
namespace gazebo {

    /**
     * \brief Obtains a parameter from sdf.
     * \param[in] sdf Pointer to the sdf object.
     * \param[in] name Name of the parameter.
     * \param[out] param Param Variable to write the parameter to.
     * \param[in] default_value Default value, if the parameter not available.
     * \param[in] verbose If true, gzerror if the parameter is not available.
     */
    template<class T>
    bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
            false) {
        if (sdf->HasElement(name)) {
            param = sdf->GetElement(name)->Get<T>();
            return true;
        } else {
            param = default_value;
            if (verbose)
                gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
        }
        return false;
    }
}
#endif /* COMMON_H */

