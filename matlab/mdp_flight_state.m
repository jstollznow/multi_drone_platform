
classdef mdp_flight_state
    enumeration
        UNKNOWN
        LANDED
        HOVERING
        MOVING
        DELETED
    end

    methods(Static)
        function State = convertstringtoflightstate(InputString)
            switch InputString
                case 'UNKNOWN'
                    State = mdp_flight_state.UNKNOWN;
                case 'LANDED'
                    State = mdp_flight_state.LANDED;
                case 'HOVERING'
                    State = mdp_flight_state.HOVERING;
                case 'MOVING'
                    State = mdp_flight_state.MOVING;
                case 'DELETED'
                    State = mdp_flight_state.DELETED;
                otherwise
                    State = mdp_flight_state.UNKNOWN;
            end
        end
    end
end