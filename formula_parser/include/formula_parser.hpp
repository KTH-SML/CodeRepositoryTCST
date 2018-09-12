#ifndef FORMULA_PARSER_HPP
#define FORMULA_PARSER_HPP

#include "exprtk.hpp"
#include <string>

template <class T>
class FormulaParser{
	std::string formula;
	std::string param_name;
	std::vector<T> var_vec;
	
	exprtk::expression<T> expr;
public:
	FormulaParser(std::string formula, std::string param_name, std::vector<T> param)
			:formula(formula), param_name(param_name), var_vec(param){

		exprtk::symbol_table<T> symbol_table;
		symbol_table.add_vector(param_name, var_vec);
		symbol_table.add_constant("pi", 3.14159);

		expr.register_symbol_table(symbol_table);

		exprtk::parser<T> parser;
		parser.compile(formula, expr);
	}

	FormulaParser(){}

	T value(std::vector<T>& param){
		var_vec = param;
		return expr.value();
	}
};

#endif
