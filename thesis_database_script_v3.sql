-- MySQL Workbench Forward Engineering

SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='TRADITIONAL,ALLOW_INVALID_DATES';

-- -----------------------------------------------------
-- Schema thesis
-- -----------------------------------------------------
DROP SCHEMA IF EXISTS `thesis` ;

-- -----------------------------------------------------
-- Schema thesis
-- -----------------------------------------------------
CREATE SCHEMA IF NOT EXISTS `thesis` DEFAULT CHARACTER SET utf8 ;
USE `thesis` ;

-- -----------------------------------------------------
-- Table `thesis`.`Node`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`Node` ;

CREATE TABLE IF NOT EXISTS `thesis`.`Node` (
  `node_id` INT NOT NULL AUTO_INCREMENT,
  `node_address_physical` INT NOT NULL,
  `node_address_logical` INT NULL DEFAULT 0,
  `node_active` TINYINT(1) NOT NULL,
  `node_name` VARCHAR(45) NULL,
  PRIMARY KEY (`node_id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `thesis`.`Port`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`Port` ;

CREATE TABLE IF NOT EXISTS `thesis`.`Port` (
  `port_id` INT NOT NULL AUTO_INCREMENT,
  `port_number` INT NOT NULL,
  `node_id` INT NOT NULL,
  PRIMARY KEY (`port_id`),
  INDEX `fk_Port_Node1_idx` (`node_id` ASC),
  CONSTRAINT `fk_Port_Node1`
    FOREIGN KEY (`node_id`)
    REFERENCES `thesis`.`Node` (`node_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `thesis`.`Port Configuration`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`Port Configuration` ;

CREATE TABLE IF NOT EXISTS `thesis`.`Port Configuration` (
  `port_configuration_id` INT NOT NULL AUTO_INCREMENT,
  `port_configuration_file` VARCHAR(250) NOT NULL,
  `port_configuration_status` VARCHAR(45) NULL,
  `data_format` VARCHAR(45) NOT NULL,
  `port_configuration_version` INT NOT NULL,
  `user_id` INT NULL DEFAULT NULL,
  `port_type` VARCHAR(45) NOT NULL,
  `data_type` CHAR(45) NOT NULL,
  `port_id` INT NOT NULL,
  PRIMARY KEY (`port_configuration_id`),
  INDEX `fk_Port Configuration_Port1_idx` (`port_id` ASC),
  CONSTRAINT `fk_Port Configuration_Port1`
    FOREIGN KEY (`port_id`)
    REFERENCES `thesis`.`Port` (`port_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `thesis`.`Node Probe Status Log`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`Node Probe Status Log` ;

CREATE TABLE IF NOT EXISTS `thesis`.`Node Probe Status Log` (
  `probe_status_id` INT NOT NULL AUTO_INCREMENT,
  `node_id` INT NOT NULL,
  `probe_time` TIMESTAMP(2) NOT NULL,
  `node_reply` TINYINT(1) NOT NULL DEFAULT 0,
  PRIMARY KEY (`probe_status_id`),
  INDEX `fk_Node Probe Status Log_Node1_idx` (`node_id` ASC),
  CONSTRAINT `fk_Node Probe Status Log_Node1`
    FOREIGN KEY (`node_id`)
    REFERENCES `thesis`.`Node` (`node_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `thesis`.`Port Value`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`Port Value` ;

CREATE TABLE IF NOT EXISTS `thesis`.`Port Value` (
  `port_id` INT NOT NULL AUTO_INCREMENT,
  `node_id` INT NOT NULL,
  `port_value` VARCHAR(45) NOT NULL,
  `time_stamp` TIMESTAMP(2) NOT NULL,
  INDEX `fk_Port Value_Node1_idx` (`node_id` ASC),
  INDEX `fk_Port Value_Port1_idx` (`port_id` ASC),
  CONSTRAINT `fk_Port Value_Node1`
    FOREIGN KEY (`node_id`)
    REFERENCES `thesis`.`Node` (`node_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `fk_Port Value_Port1`
    FOREIGN KEY (`port_id`)
    REFERENCES `thesis`.`Port` (`port_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `thesis`.`command`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `thesis`.`command` ;

CREATE TABLE IF NOT EXISTS `thesis`.`command` (
  `command_id` INT NOT NULL AUTO_INCREMENT,
  `command_code` INT NOT NULL,
  `node_id` INT NOT NULL,
  `time_stamp` DATETIME NOT NULL,
  PRIMARY KEY (`command_id`),
  INDEX `fk_command_Node1_idx` (`node_id` ASC),
  CONSTRAINT `fk_command_Node1`
    FOREIGN KEY (`node_id`)
    REFERENCES `thesis`.`Node` (`node_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;
