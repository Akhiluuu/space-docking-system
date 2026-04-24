#pragma once

#include "Scenario.h"
#include <tinyxml2.h>
#include <string>
#include <memory>

/**
 * Loads scenario definitions from XML files.
 * Uses tinyxml2 for XML parsing.
 */
class ScenarioLoader {
public:
    /**
     * Load a scenario from an XML file
     * @param filepath Path to the XML scenario file
     * @return Shared pointer to the loaded Scenario
     * @throws std::runtime_error if file cannot be loaded or parsed
     */
    static std::shared_ptr<Scenario> loadFromFile(const std::string& filepath);
    
private:
    /**
     * Parse simulation configuration from XML element
     */
    static SimulationConfig parseSimulationConfig(const tinyxml2::XMLElement* simElement);
    
    /**
     * Parse platform parameters from XML element
     */
    static PlatformParameters parsePlatformParameters(const tinyxml2::XMLElement* paramsElement);
    
    /**
     * Parse all contact events from the events section
     */
    static std::vector<ContactEvent> parseEvents(const tinyxml2::XMLElement* eventsElement);
    
    /**
     * Parse a single contact event
     */
    static ContactEvent parseContactEvent(const tinyxml2::XMLElement* contactElement);
};
